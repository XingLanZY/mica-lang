#include "codegen.h"

#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "proto.h"
#include "gc.h"
#include "regalloc.h"
#include "table.h"
#include "peephole.h"

/* ============================================================
   AST → 字节码（Proto）的编译器。

   ## 寄存器模型
   - 每帧寄存器 = 栈的一段。slot 0 保留（帧内函数值）；参数 1..arity；
     局部变量与临时共用 reg_top。编译表达式"净零临时"：进入时保存 reg_top，
     退出时恢复（表达式求值的临时槽生命周期限定在表达式内）。
   - 局部变量：固定槽，块退出回滚复用。临时：语句级栈式分配。

   ## 闭包/开放捕获值（open capvalue）
   - 仅 lambda 捕获外层变量；func（含匿名）不捕获。
   - 每函数先做"捕获预扫描"（scan_*），收集嵌套 lambda 引用的自由名，
     add_local 据此标记 is_captured。被捕获局部**仍是普通寄存器**（不装箱）：
     声明函数读写走普通指令；运行时 OP_CLOSURE 才建 ObjCapVal 指向该栈槽（懒装箱）。
   - 由于开放捕获值指向栈槽，被捕获局部的槽**保留到帧返回**（cap_floor 下水位，
     restore_regs/end_scope 钳制 reg_top），避免块退出后槽被复用导致捕获值指向别处。
   - lambda 访问捕获变量走 GET/SET_CAPVAL（closure.capvals，即捕获值）；创建 lambda 走 OP_CLOSURE。

   ## 关键不变量
   - 数组/Map 字面量与多返回值结果槽必须"先分配全部槽"再编译元素，
     否则嵌套字面量的临时会与既有槽碰撞。
   - 简单赋值经 store_name（而非直写槽），确保 lambda 捕获的变量写进寄存器槽。
   ============================================================ */

typedef struct Local {
    const char *name;   /* AST arena 字符串 */
    int slot;
    bool is_const;
    int depth;
    bool is_captured;   /* 被嵌套 lambda 捕获 → 槽保留到帧返回（开放捕获值指向该栈槽） */
    bool computed;      /* let name => body 计算属性：读取时自动调用 */
} Local;

typedef struct LoopInfo {
    int *breaks;
    size_t n_breaks, cap_breaks;
    int *continues;
    size_t n_cont, cap_cont;
    int continue_target;   /* 循环末尾补丁时填写的绝对指令下标 */
    int defer_base;        /* 循环入口时 n_active_defers（break/continue 需弹出到该层） */
    struct LoopInfo *enclosing;
} LoopInfo;

/* 捕获描述：lambda 的 cell 从何而来 */
typedef struct {
    bool is_local;   /* true=外层函数的局部槽（index=槽号）；false=外层闭包的 capval（index=下标） */
    int index;
    const char *name;   /* 去重用 */
} Capval;

/* 名字集合（捕获预扫描 / 已收集的捕获名） */
typedef struct {
    const char **names;
    size_t n, cap;
} NameSet;

/* import 名 → 全限定全局名（方案 A 压平）：sqrt→"System.Math.sqrt"、Math→"System.Math"。
   编译期记录，codegen_ident/codegen_member 对 import 来的名字发全限定 LOAD_GLOBAL，
   消除裸全局键（跨程序集/跨导入碰撞点）。qualified 是 malloc 拷贝（free_import_map 释放）。 */
typedef struct {
    const char **names;
    const char **qualified;
    size_t n, cap;
} ImportMap;

/* 错误集（编译期元信息）：`error IoError { Case(payload...), ... }` */
typedef struct {
    const char *name;
    const AstErrorCase *cases;
    size_t n_cases;
} ErrorSetInfo;

/* 枚举（编译期元信息）：`enum Color { Red(rgb), Green, ... }` */
typedef struct {
    const char *name;
    const AstEnumMember *members;   /* 全部成员（含无 payload） */
    size_t n_members;
} EnumSetInfo;

/* 已建好的类（编译期 permanent ObjClass） */
typedef struct {
    const char *name;
    ObjClass *klass;
    const AstNode *ast;      /* 类声明 AST（命名参数解析方法/init 参数用） */
    bool nested;             /* 嵌套类（函数体内声明）：标识符引用发 OP_CONST 类常量（非 LOAD_GLOBAL） */
} ClassEntry;

/* 顶层函数签名（命名参数静态重排用） */
typedef struct {
    const char *name;
    const AstParam *params;
    size_t n_params;
    bool has_varargs;
    bool is_throws;   /* 编译期确认：直接调用须被 try/catch 保护 */
} FuncSig;

/* 局部变量 → 类 的简单推断（`var x = ClassName(...)` → x 是 ClassName 实例） */
typedef struct {
    const char *var;
    const char *class_name;
} VarClass;

/* do-catch 错误捕获栈：do 块内 try/throw 的错误分支不 OP_RETURN/OP_THROW，
   而是把错误值搬入 err_reg 并 JMP 到 catch 入口（catch 入口确定后统一 patch）。 */
typedef struct CatchSite {
    int err_reg;              /* 错误值约定寄存器 */
    size_t *jumps;            /* 错误分支的 JMP 位置（do_catch 编译完 body 后 patch 到 catch 入口） */
    size_t n_jumps, cap_jumps;
    struct CatchSite *prev;
} CatchSite;

struct Codegen {
    Gc *gc;
    PeepholeState *peep;      /* 窥孔优化器状态（runtime 持有，见 A3） */
    struct Codegen *enclosing;
    Proto *proto;
    ObjString *func_name;   /* permanent */
    Local *locals;
    size_t n_locals, cap_locals;
    int scope_depth;
    int reg_top;
    int max_regs;
    int cap_floor;          /* 被捕获局部槽的下水位：reg_top 不得回退到其下（开放捕获值指向栈槽，须保留到帧返回） */
    int cur_line;           /* 最近 emit 的行号（error(c,NULL,...) 的位置回退，避免报 line 0） */
    LoopInfo *loops;
    bool has_error;
    bool is_lambda;         /* lambda 捕获外层；func 不捕获 */
    bool is_vararg;         /* 本函数声明了 ...（vararg）：body 内 ... 是多余实参数组 */
    int varargs_slot;       /* vararg 函数预留的实参数组槽（slot 1+arity，-1=非 vararg） */
    NameSet captured_names; /* 本函数被嵌套 lambda 捕获的局部名（装箱用） */
    Capval *capvals;
    int n_capvals, cap_capvals;
    const char *module_prefix;   /* 模块编译：内部全局加 "prefix." 前缀 */
    NameSet module_internal;     /* 模块内部声明名（须加前缀） */
    NameSet imported_modules;    /* 整体 import 的模块名（命名空间成员函数调用不注入 receiver） */
    ImportMap import_map;        /* import 名 → 全限定全局名（方案 A 压平） */
    NameSet computed_globals;    /* 顶层计算属性名（let r => body）：读取时自动调用 */
    ErrorSetInfo *error_sets;    /* 本函数作用域内声明的错误集 */
    size_t n_error_sets, cap_error_sets;
    EnumSetInfo *enums;          /* 本函数作用域内声明的枚举（payload 构造/is/match 用） */
    size_t n_enums, cap_enums;
    ClassEntry *classes;         /* 顶层已建类（本函数 + enclosing 链共享查找） */
    size_t n_classes, cap_classes;
    FuncSig *func_sigs;          /* 顶层函数签名（本函数 + enclosing 链共享；命名参数用） */
    size_t n_func_sigs, cap_func_sigs;
    VarClass *var_classes;       /* 局部 var → 类 推断（实例方法命名参数用） */
    size_t n_var_classes, cap_var_classes;
    ObjClass *cur_class;     /* 当前编译的方法所属类（super 用） */
    const char *self_name;   /* 当前方法 self 参数名（super 加载 self 用） */
    int n_active_defers;     /* 活动 defer 块标记数（break/continue 需弹出） */
    int protect_depth;       /* >0 表示当前调用点被 try/catch 保护（throws 调用豁免） */
    struct CatchSite *catch_chain; /* 最近 do-catch 的捕获上下文（块内 try/throw 错误重定向） */
    CodegenError *err;
};
typedef struct Codegen Codegen;

/* ---------- 名字集合 ---------- */

static bool name_set_has(const NameSet *s, const char *name) {
    for (size_t i = 0; i < s->n; i++) {
        if (strcmp(s->names[i], name) == 0) return true;
    }
    return false;
}

static void name_set_add(NameSet *s, const char *name) {
    if (name_set_has(s, name)) return;
    if (s->n == s->cap) {
        size_t ncap = s->cap ? s->cap * 2 : 16;
        s->names = realloc((void *)s->names, ncap * sizeof(const char *));
        assert(s->names != NULL);
        s->cap = ncap;
    }
    s->names[s->n++] = name;
}

static void name_set_free(NameSet *s) {
    free((void *)s->names);
    s->names = NULL;
    s->n = s->cap = 0;
}

/* ---------- import 名 → 全限定全局名（方案 A 压平） ---------- */

static const char *import_map_get(Codegen *c, const char *name) {
    for (Codegen *cc = c; cc; cc = cc->enclosing) {
        for (size_t i = 0; i < cc->import_map.n; i++) {
            if (strcmp(cc->import_map.names[i], name) == 0) return cc->import_map.qualified[i];
        }
    }
    return NULL;
}

static void import_map_add(Codegen *c, const char *name, const char *qualified) {
    if (import_map_get(c, name)) return; /* 已有（含 enclosing 链） */
    if (c->import_map.n == c->import_map.cap) {
        size_t ncap = c->import_map.cap ? c->import_map.cap * 2 : 8;
        c->import_map.names = realloc((void *)c->import_map.names, ncap * sizeof(const char *));
        c->import_map.qualified = realloc(c->import_map.qualified, ncap * sizeof(const char *));
        assert(c->import_map.names && c->import_map.qualified);
        c->import_map.cap = ncap;
    }
    const size_t qlen = strlen(qualified);
    char *q = malloc(qlen + 1);
    assert(q != NULL);
    memcpy(q, qualified, qlen + 1); /* 免 strcpy 的 C4996 警告 */
    c->import_map.names[c->import_map.n] = name;
    c->import_map.qualified[c->import_map.n] = q;
    c->import_map.n++;
}

static void free_import_map(ImportMap *m) {
    for (size_t i = 0; i < m->n; i++) free((void *)m->qualified[i]);
    free((void *)m->names);
    free((void *)m->qualified);
    m->names = NULL;
    m->qualified = NULL;
    m->n = m->cap = 0;
}

/* ---------- 错误集（编译期元信息） ---------- */

static ErrorSetInfo *find_error_set(Codegen *c, const char *name) {
    for (Codegen *cc = c; cc; cc = cc->enclosing) {
        for (size_t i = 0; i < cc->n_error_sets; i++) {
            if (strcmp(cc->error_sets[i].name, name) == 0) return &cc->error_sets[i];
        }
    }
    return NULL;
}

static const AstErrorCase *find_error_case(const ErrorSetInfo *set, const char *case_name) {
    for (size_t i = 0; i < set->n_cases; i++) {
        if (strcmp(set->cases[i].name, case_name) == 0) return &set->cases[i];
    }
    return NULL;
}

static void register_error_set(Codegen *c, const AstNode *n) {
    if (c->n_error_sets == c->cap_error_sets) {
        size_t ncap = c->cap_error_sets ? c->cap_error_sets * 2 : 4;
        c->error_sets = realloc(c->error_sets, ncap * sizeof(ErrorSetInfo));
        assert(c->error_sets != NULL);
        c->cap_error_sets = ncap;
    }
    c->error_sets[c->n_error_sets].name = n->as.error_set.name;
    c->error_sets[c->n_error_sets].cases = n->as.error_set.cases;
    c->error_sets[c->n_error_sets].n_cases = n->as.error_set.n_cases;
    c->n_error_sets++;
}

/* ---------- 枚举（编译期元信息） ---------- */

static EnumSetInfo *find_enum(Codegen *c, const char *name) {
    for (Codegen *cc = c; cc; cc = cc->enclosing) {
        for (size_t i = 0; i < cc->n_enums; i++) {
            if (strcmp(cc->enums[i].name, name) == 0) return &cc->enums[i];
        }
    }
    return NULL;
}

/* 返回枚举成员指针；带 payload（n_payload>0）时 case_name 匹配，否则 NULL。 */
static const AstEnumMember *find_enum_case(const EnumSetInfo *set, const char *case_name) {
    for (size_t i = 0; i < set->n_members; i++) {
        if (strcmp(set->members[i].name, case_name) == 0) return &set->members[i];
    }
    return NULL;
}

static void register_enum(Codegen *c, const AstNode *n) {
    if (c->n_enums == c->cap_enums) {
        size_t ncap = c->cap_enums ? c->cap_enums * 2 : 4;
        c->enums = realloc(c->enums, ncap * sizeof(EnumSetInfo));
        assert(c->enums != NULL);
        c->cap_enums = ncap;
    }
    c->enums[c->n_enums].name = n->as.enum_.name;
    c->enums[c->n_enums].members = n->as.enum_.members;
    c->enums[c->n_enums].n_members = n->as.enum_.n_members;
    c->n_enums++;
}

/* ---------- 类表 ---------- */

static ObjClass *find_class(Codegen *c, const char *name) {
    for (Codegen *cc = c; cc; cc = cc->enclosing) {
        for (size_t i = 0; i < cc->n_classes; i++) {
            if (strcmp(cc->classes[i].name, name) == 0) return cc->classes[i].klass;
        }
    }
    return NULL;
}

/* 返回 ClassEntry（含 AST，命名参数解析方法参数用），未找到返回 NULL。 */
static ClassEntry *find_class_entry(Codegen *c, const char *name) {
    for (Codegen *cc = c; cc; cc = cc->enclosing) {
        for (size_t i = 0; i < cc->n_classes; i++) {
            if (strcmp(cc->classes[i].name, name) == 0) return &cc->classes[i];
        }
    }
    return NULL;
}

static void add_class(Codegen *c, const char *name, ObjClass *klass, const AstNode *ast, bool nested) {
    if (c->n_classes == c->cap_classes) {
        size_t ncap = c->cap_classes ? c->cap_classes * 2 : 8;
        c->classes = realloc(c->classes, ncap * sizeof(ClassEntry));
        assert(c->classes != NULL);
        c->cap_classes = ncap;
    }
    c->classes[c->n_classes].name = name;
    c->classes[c->n_classes].klass = klass;
    c->classes[c->n_classes].ast = ast;
    c->classes[c->n_classes].nested = nested;
    c->n_classes++;
}

/* 顶层函数签名（本函数 + enclosing 链共享查找）。 */
static const FuncSig *find_func_sig(Codegen *c, const char *name) {
    for (Codegen *cc = c; cc; cc = cc->enclosing) {
        for (size_t i = 0; i < cc->n_func_sigs; i++) {
            if (strcmp(cc->func_sigs[i].name, name) == 0) return &cc->func_sigs[i];
        }
    }
    return NULL;
}

static void add_func_sig(Codegen *c, const char *name, const AstParam *params, size_t n_params,
                         bool has_varargs, bool is_throws) {
    if (c->n_func_sigs == c->cap_func_sigs) {
        size_t ncap = c->cap_func_sigs ? c->cap_func_sigs * 2 : 16;
        c->func_sigs = realloc(c->func_sigs, ncap * sizeof(FuncSig));
        assert(c->func_sigs != NULL);
        c->cap_func_sigs = ncap;
    }
    c->func_sigs[c->n_func_sigs].name = name;
    c->func_sigs[c->n_func_sigs].params = params;
    c->func_sigs[c->n_func_sigs].n_params = n_params;
    c->func_sigs[c->n_func_sigs].has_varargs = has_varargs;
    c->func_sigs[c->n_func_sigs].is_throws = is_throws;
    c->n_func_sigs++;
}

/* 局部 var → 类 推断（本函数作用域）。 */
static const char *find_var_class(Codegen *c, const char *var) {
    for (Codegen *cc = c; cc; cc = cc->enclosing) {
        for (size_t i = 0; i < cc->n_var_classes; i++) {
            if (strcmp(cc->var_classes[i].var, var) == 0) return cc->var_classes[i].class_name;
        }
    }
    return NULL;
}

static void add_var_class(Codegen *c, const char *var, const char *class_name) {
    if (c->n_var_classes == c->cap_var_classes) {
        size_t ncap = c->cap_var_classes ? c->cap_var_classes * 2 : 8;
        c->var_classes = realloc(c->var_classes, ncap * sizeof(VarClass));
        assert(c->var_classes != NULL);
        c->cap_var_classes = ncap;
    }
    c->var_classes[c->n_var_classes].var = var;
    c->var_classes[c->n_var_classes].class_name = class_name;
    c->n_var_classes++;
}

/* ---------- 错误 ---------- */

/* 格式串编译期检查（同 vm.h 的 MICA_PRINTF 思路）：变参函数写错实参不会有任何提示，
   而这些错误都出现在"函数太大/常量太多"这类**硬上限诊断**里，写错就等于诊断不可用。 */
#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 3, 4)))
#endif
static void error(Codegen *c, const AstNode *n, const char *fmt, ...) {
    if (c->has_error) return;
    c->has_error = true;
    if (c->err) {
        /* 无节点位置时回退到"最近发射的指令行号"（emit 维护 c->cur_line）：
           旧实现直接写 0 → 硬上限类错误报 "line 0, col 0"，用户无从定位。 */
        c->err->line = n ? n->line : c->cur_line;
        c->err->col = n ? n->col : 0;
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(c->err->message, sizeof(c->err->message), fmt, ap);
        va_end(ap);
    }
}

/* ---------- 发射 ---------- */

static void emit(Codegen *c, OpCode op, int a, int b, int cc, int line) {
    if (line > 0) c->cur_line = line; /* error(c, NULL, ...) 的位置回退源 */
    /* 字段位宽守卫（2026-08-15：ABC 的 a/b 8 位、c 9 位；AD 的 d 16 位不受限）：
       常量池/标识符池可到 512+，但 b 字段 8 位只能引用 ≤255，c 字段 9 位 ≤511。
       ADDVN 族/GETI/SETI 由各自发射路径先做 k<512 回退，不落此守卫；
       GLOAD_INDEX 全局下标 >255 在 codegen_index_get 回退 LOAD_GLOBAL+INDEX。 */
    switch (op) {
        case OP_CLOSURE: case OP_MATCH_LITERAL: case OP_IS_TYPE:
            if (b >= 256 || cc >= 512) {
                error(c, NULL, "too many constants in function (limit 256 in 8-bit b / 512 in 9-bit c)");
                return;
            }
            break;
        /* 这四条把常量/标识符下标放在 8 位字段（k_instr_desc 的 IARG_CONST）：
           池上限是 512，下标 ≥256 会静默截断（实测 300 个函数后 match 的错误消息变空）。 */
        case OP_ERROR: case OP_ERROR_INFO:
            if (a >= 256) {
                error(c, NULL, "too many constants in function (8-bit constant index)");
                return;
            }
            break;
        case OP_ENUM_INFO:
            if (a >= 256 || b >= 256) {
                error(c, NULL, "too many constants in function (8-bit constant index)");
                return;
            }
            break;
        case OP_GET_MEMBER: case OP_SET_MEMBER: case OP_GET_METHOD:
            if (b >= 256) {
                error(c, NULL, "too many member names in function (limit 256)");
                return;
            }
            break;
        default:
            break;
    }
    proto_emit(c->proto, op, a, b, cc, line);
}

/* 跳转发射：条件寄存器进 A，偏移进 sBx（AD：d 字段 16 位）。offset=0 表示占位待 patch。 */
static void emit_jump(Codegen *c, OpCode op, int cond, int offset, int line) {
    proto_emit(c->proto, op, cond, offset & 0xffff, 0, line);
}

static void patch_jump(Codegen *c, size_t idx, size_t target) {
    /* sBx 是 16 位有符号（AD 的 d 字段）：函数体过大时偏移会静默截断，跳转落到错误位置
       （实测 >32767 的循环回边直接访问违例）。必须显式报错，而不是让截断发生。 */
    const long long off = (long long)target - (long long)idx - 1;
    if (off < -32768 || off > 32767) {
        error(c, NULL,
              "function too large: jump distance %lld exceeds 16-bit sBx range "
              "(split the function or reduce the loop body)",
              off);
        return;
    }
    proto_patch_jump(c->proto, idx, (int)off);
}

/* 整数常量装载：16 位有符号（[-32768, 32767]）直接用 MOVI 立即数（不占常量池），
   超出范围回退 OP_KNUM（大整数才进常量池）。 */
static int add_const(Codegen *c, Value v);   /* 定义在下方 */
static void emit_int_const(Codegen *c, int dest, int64_t v, int line) {
    if (instr_movi_fits(v)) {
        proto_emit(c->proto, OP_MOVI, dest, (int)v, 0, line); /* MOVI：AD，d=16 位立即数 */
    } else {
        emit(c, OP_KNUM, dest, add_const(c, val_float((double)v)), 0, line); /* 大整数 → 装箱 KNUM */
    }
}

/* 阶段 F：按常量类型分派装载（KNUM 数值 / KSTR 字符串 / OP_CONST 其它），
   KNUM 优先内嵌 imm，否则落数值池；KSTR 走字符串池。
   常量池按类型分三区（num/str/generic），add_const 按值类型路由到对应池。 */
static void emit_const_op(Codegen *c, int dest, Value v, int line) {
    if (val_is_num(v)) {
        const int k = add_const(c, v);
        if (k >= 0) emit(c, OP_KNUM, dest, k, 0, line);
    } else if (val_is_string(v)) {
        const int k = add_const(c, v);
        if (k >= 0) emit(c, OP_KSTR, dest, k, 0, line);
    } else {
        const int k = add_const(c, v);
        if (k >= 0) emit(c, OP_CONST, dest, k, 0, line);
    }
}

/* 给生成的函数原型标记所属模块（module_prefix，非模块为 NULL）。
   标准库内部校验（System.Builtin 仅 System.* 模块可调）依赖此归属。 */
static void set_func_module(Gc *gc, const char *module_prefix, Proto *proto) {
    if (module_prefix) {
        proto->module = obj_string_new_permanent(gc, module_prefix, strlen(module_prefix));
    }
}

/* 图着色寄存器分配后处理：压缩 max_regs（失败/无改善则保持原样），再跑窥孔优化。 */
static void finish_func(Proto *proto, PeepholeState *ps) {
    const char *no_ra = getenv("MICA_NO_REGALLOC");
    if (!(no_ra && no_ra[0] == '1')) {
        const int mr = regalloc_run(proto, proto->arity, proto->max_regs);
        if (mr >= 0 && (uint8_t)mr < proto->max_regs) proto->max_regs = (uint8_t)mr;
    }
    peephole_run(proto, ps); /* 窥孔优化（状态由 runtime 持有；可开关） */
}

/* 添加常量并按类型路由到对应池（2026-08-10 分池）：
   num→num_consts（OP_KNUM/ADDVN/GETI/ISEQN 读）、str→str_consts（OP_KSTR/ISEQS 读）、
   其它→通用 constants（OP_CONST/闭包/类/bool/null 读）。溢出时报编译错误。 */
static int add_const(Codegen *c, Value v) {
    int k;
    if (val_is_num(v)) k = proto_add_num_const(c->proto, v);
    else if (val_is_string(v)) k = proto_add_str_const(c->proto, v);
    else k = proto_add_const(c->proto, v);
    if (k < 0) error(c, NULL, "too many constants in function");
    return k;
}

/* 强制进通用 constants 池（OP_CONST/MATCH_LITERAL/IS_TYPE/ERROR 等读通用池的指令；
   即便值是 num/str 也进通用池——因为目标指令只读 constants 数组）。 */
static int add_const_gen(Codegen *c, Value v) {
    const int k = proto_add_const(c->proto, v);
    if (k < 0) error(c, NULL, "too many constants in function");
    return k;
}

/* 标识符池内联（成员/全局/方法名）：同内容复用同一下标；返回池下标。
   vm 的 OP_GET/SET_MEMBER、OP_GET_METHOD、OP_LOAD/STORE_GLOBAL 从池读 SSO String。 */
static int intern_name(Codegen *c, const char *name) {
    const int k = proto_add_ident(c->proto, name, strlen(name));
    if (k < 0) error(c, NULL, "too many identifiers in function");
    return k;
}

/* 常量池字符串（错误消息、错误集/case 名等非标识符场合）：返回通用池下标。
   这些字符串由 OP_ERROR/OP_ERROR_INFO/OP_ENUM_INFO 经 const_str_or_null 读通用池，
   故强制进 constants（不走 str_consts——那是 OP_KSTR/ISEQS 专用）。 */
static int intern_const(Codegen *c, const char *name) {
    ObjString *s = obj_string_new_permanent(c->gc, name, strlen(name));
    return add_const_gen(c, obj_string_val(s));
}

/* 全局标识符池内联（get_global 系列：LOAD_GLOBAL/STORE_GLOBAL/GLOAD_INDEX 的 d/b 下标）。
   与 intern_name（成员名）分离——全局名独立配额（PROTO_MAX_GLOBAL_IDENTS=65536）。 */
static int global_ident(Codegen *c, const char *name) {
    const int k = proto_add_global_ident(c->proto, name, strlen(name));
    if (k < 0) error(c, NULL, "too many global names in function");
    return k;
}

/* 全局名：模块内部声明加 "module.name" 前缀；import/内置/外部用原名。入全局标识符池。 */
static int global_name(Codegen *c, const char *name) {
    if (c->module_prefix && name_set_has(&c->module_internal, name)) {
        char buf[512];
        snprintf(buf, sizeof(buf), "%s.%s", c->module_prefix, name);
        return global_ident(c, buf);
    }
    return global_ident(c, name);
}

/* ---------- 寄存器与作用域 ---------- */

static int alloc_temp(Codegen *c) {
    int r = c->reg_top++;
    if (c->reg_top > c->max_regs) c->max_regs = c->reg_top;
    /* >=：max_regs 是 uint8_t（上限 255），reg_top 达 256 会截断成 0 → VM 帧 0 寄存器、
       指令写 R[] 越界。256 寄存器同时存活是病态，直接编译报错更安全。 */
    if (c->reg_top >= PROTO_MAX_REGS) error(c, NULL, "too many registers in function");
    return r;
}

static int add_local(Codegen *c, const char *name, bool is_const) {
    int slot = c->reg_top++;
    if (c->reg_top > c->max_regs) c->max_regs = c->reg_top;
    if (c->reg_top >= PROTO_MAX_REGS) error(c, NULL, "too many registers in function");
    if (c->n_locals == c->cap_locals) {
        size_t ncap = c->cap_locals ? c->cap_locals * 2 : 16;
        c->locals = realloc(c->locals, ncap * sizeof(Local));
        assert(c->locals != NULL);
        c->cap_locals = ncap;
    }
    const bool captured = name_set_has(&c->captured_names, name);
    c->locals[c->n_locals++] = (Local){name, slot, is_const, c->scope_depth, captured};
    return slot;
}

static Local *find_local(Codegen *c, const char *name) {
    for (size_t i = c->n_locals; i-- > 0;) {
        if (c->locals[i].name && strcmp(c->locals[i].name, name) == 0) {
            return &c->locals[i];
        }
    }
    return NULL;
}

static int resolve_local(const Codegen *c, const char *name) {
    Local *l = find_local((Codegen *)c, name);
    return l ? l->slot : -1;
}

/* 计算属性判定：当前或外层作用域的名字是否是 let r => body（读取时自动调用）。 */
static bool local_computed(const Codegen *c, const char *name) {
    Local *l = find_local((Codegen *)c, name);
    return l && l->computed;
}
static bool name_is_computed(Codegen *c, const char *name) {
    for (Codegen *cc = c; cc; cc = cc->enclosing) {
        if (local_computed(cc, name)) return true;
        if (name_set_has(&cc->computed_globals, name)) return true;
    }
    return false;
}
static void mark_local_computed(Codegen *c, const char *name) {
    Local *l = find_local(c, name);
    if (l) l->computed = true;
}

/* 读取计算属性：闭包已在 dest，调用它（0 参）并把结果覆写 dest。
   统一约定：slot1=null self，argc 恒含 self。 */
static void emit_computed_call(Codegen *c, const AstNode *n, int dest) {
    emit(c, OP_NULL, dest + 1, 0, 0, n->line); /* null self */
    emit(c, OP_CALL, dest, 1, RESULT_VARARGS, n->line);
}

/* 若表达式值已在某局部寄存器（无需复制即可直接作操作数），返回 true 并写 *reg。
   仅**非捕获**局部可直用——被捕获局部是开放捕获值，嵌套调用可能经 capval 修改它，
   直接别名会读到修改后的值（求值顺序改变）。capval/全局/复杂表达式须复制到临时。
   用于 member/index/二元 访问省掉操作数的冗余 MOV（这些指令的操作数可指向任意寄存器）。 */
static bool expr_plain_reg(Codegen *c, const AstNode *n, int *reg) {
    if (n->kind == AST_IDENT) {
        Local *l = find_local(c, n->as.ident.name);
        if (l && !l->is_captured) { *reg = l->slot; return true; }
    }
    return false;
}

/* 添加捕获描述到当前 lambda（按名字去重），返回 capval 下标。 */
static int add_capval(Codegen *c, const char *name, bool is_local, int index) {
    for (int i = 0; i < c->n_capvals; i++) {
        if (c->capvals[i].name && strcmp(c->capvals[i].name, name) == 0) return i;
    }
    if (c->n_capvals == c->cap_capvals) {
        int ncap = c->cap_capvals ? c->cap_capvals * 2 : 4;
        c->capvals = realloc(c->capvals, ncap * sizeof(Capval));
        assert(c->capvals != NULL);
        c->cap_capvals = ncap;
    }
    c->capvals[c->n_capvals++] = (Capval){is_local, index, name};
    return c->n_capvals - 1;
}

/* 解析当前 lambda 对外层变量的捕获：找到外层局部槽或外层闭包 capval。
   仅 lambda 捕获；func 不捕获。返回 capval 下标，-1=未找到（按全局处理）。 */
static int resolve_capval(Codegen *c, const char *name) {
    if (!c->is_lambda || c->enclosing == NULL) return -1;
    const int slot = resolve_local(c->enclosing, name);
    if (slot >= 0) {
        return add_capval(c, name, true, slot);
    }
    const int cap = resolve_capval(c->enclosing, name);
    if (cap >= 0) return add_capval(c, name, false, cap);
    return -1;
}

/* ---------- 捕获预扫描 ----------
   扫描函数体，收集嵌套 lambda 引用的"自由名"（非 lambda 自身局部）。
   depth==0 表示程序顶层（var=全局，不入 scope）；函数体内 var 是局部。
   装箱判定在 add_local 时用 captured_names 匹配。 */

static void scan_stmt(const AstNode *n, NameSet *scope, NameSet *captured, int depth,
                      const char *selfn);
static void scan_expr(const AstNode *n, NameSet *scope, NameSet *captured, int depth,
                      const char *selfn);

static void scan_body(const AstNode *body, NameSet *scope, NameSet *captured, const char *selfn) {
    if (!body) return;
    if (body->kind == AST_BLOCK) {
        size_t mark = scope->n;
        for (size_t i = 0; i < body->as.block.n; i++)
            scan_stmt(body->as.block.stmts[i], scope, captured, 1, selfn);
        scope->n = mark;
    } else {
        scan_expr(body, scope, captured, 1, selfn);
    }
}

static void scan_stmt(const AstNode *n, NameSet *scope, NameSet *captured, int depth,
                      const char *selfn) {
    if (!n) return;
    switch (n->kind) {
        case AST_PROP_DECL: {
            /* 计算属性体是 0 参闭包：其自由名（非本作用域）是捕获候选 */
            NameSet lscope = {0};
            scan_body(n->as.prop_decl.body, &lscope, captured, selfn);
            name_set_free(&lscope);
            break;
        }
        case AST_VAR_DECL:
            if (depth > 0) { /* 顶层 var=全局，不入局部作用域 */
                for (size_t i = 0; i < n->as.var_decl.n_names; i++) {
                    name_set_add(scope, n->as.var_decl.names[i]);
                }
            }
            for (size_t i = 0; i < n->as.var_decl.n_values; i++) {
                scan_expr(n->as.var_decl.values[i], scope, captured, depth, selfn);
            }
            break;
        case AST_ASSIGN:
            for (size_t i = 0; i < n->as.assign.n; i++) scan_expr(n->as.assign.targets[i], scope, captured, depth, selfn);
            for (size_t i = 0; i < n->as.assign.n_values; i++) scan_expr(n->as.assign.values[i], scope, captured, depth, selfn);
            break;
        case AST_EXPR_STMT:
            scan_expr(n->as.expr_stmt.expr, scope, captured, depth, selfn);
            break;
        case AST_BLOCK: {
            size_t mark = scope->n;
            for (size_t i = 0; i < n->as.block.n; i++) scan_stmt(n->as.block.stmts[i], scope, captured, depth + 1, selfn);
            scope->n = mark;
            break;
        }
        case AST_DEFER: {
            NameSet lscope = {0};
            scan_body(n->as.defer_.block, &lscope, captured, selfn);
            name_set_free(&lscope);
            break;
        }
        case AST_IF:
            scan_expr(n->as.if_.cond, scope, captured, depth, selfn);
            scan_stmt(n->as.if_.then_, scope, captured, depth + 1, selfn);
            for (size_t i = 0; i < n->as.if_.n_elif; i++) {
                scan_expr(n->as.if_.elif_conds[i], scope, captured, depth, selfn);
                scan_stmt(n->as.if_.elif_bodies[i], scope, captured, depth + 1, selfn);
            }
            if (n->as.if_.else_) scan_stmt(n->as.if_.else_, scope, captured, depth + 1, selfn);
            break;
        case AST_WHILE:
            scan_expr(n->as.while_.cond, scope, captured, depth, selfn);
            scan_stmt(n->as.while_.body, scope, captured, depth + 1, selfn);
            break;
        case AST_FOR_IN: {
            size_t mark = scope->n;
            for (size_t i = 0; i < n->as.for_in.n_vars; i++) name_set_add(scope, n->as.for_in.vars[i]);
            scan_expr(n->as.for_in.iterable, scope, captured, depth, selfn);
            if (n->as.for_in.step) scan_expr(n->as.for_in.step, scope, captured, depth, selfn);
            scan_stmt(n->as.for_in.body, scope, captured, depth + 1, selfn);
            scope->n = mark;
            break;
        }
        case AST_RETURN:
            for (size_t i = 0; i < n->as.return_.n; i++) scan_expr(n->as.return_.values[i], scope, captured, depth, selfn);
            break;
        case AST_THROW:
            scan_expr(n->as.throw_.expr, scope, captured, depth, selfn);
            break;
        case AST_TRY_BLOCK:
            scan_stmt(n->as.try_block.body, scope, captured, depth, selfn);
            break;
        case AST_DO_CATCH: {
            scan_stmt(n->as.do_catch.body, scope, captured, depth, selfn);
            const CatchHandler *h = &n->as.do_catch.handler;
            if (h->kind == CATCH_BLOCK) {
                size_t mark = scope->n;
                if (h->error_name) name_set_add(scope, h->error_name);
                scan_stmt(h->fallback, scope, captured, depth + 1, selfn);
                scope->n = mark;
            } else if (h->kind == CATCH_FALLBACK) {
                scan_expr(h->fallback, scope, captured, depth, selfn);
            } else {
                for (size_t i = 0; i < h->n_arms; i++) {
                    size_t mark = scope->n;
                    for (size_t j = 0; j < h->arms[i].n_payload; j++)
                        name_set_add(scope, h->arms[i].payload[j]);
                    const AstNode *av = h->arms[i].value;
                    if (av && av->kind == AST_BLOCK) scan_stmt(av, scope, captured, depth + 1, selfn);
                    else scan_expr(av, scope, captured, depth, selfn);
                    scope->n = mark;
                }
            }
            break;
        }
        case AST_MATCH:
            scan_expr(n, scope, captured, depth, selfn);
            break;
        case AST_BLOCK_EXPR: {
            size_t mark = scope->n;
            for (size_t i = 0; i < n->as.block.n; i++) scan_stmt(n->as.block.stmts[i], scope, captured, depth + 1, selfn);
            scope->n = mark;
            break;
        }
        case AST_FUNC_DECL:
            /* func 不捕获：其体不贡献外层捕获 */
            break;
        default:
            break;
    }
}

static void scan_expr(const AstNode *n, NameSet *scope, NameSet *captured, int depth,
                      const char *selfn) {
    if (!n) return;
    switch (n->kind) {
        case AST_NUMBER:
        case AST_STRING:
        case AST_NULL:
        case AST_TRUE:
        case AST_FALSE:
        case AST_UNINITIALIZED:
        case AST_VARARGS:
            break;
        case AST_IDENT:
            if (!name_set_has(scope, n->as.ident.name)) name_set_add(captured, n->as.ident.name);
            break;
        case AST_ERROR_VAL:
            scan_expr(n->as.error_val.msg, scope, captured, depth, selfn);
            break;
        case AST_ARRAY:
            for (size_t i = 0; i < n->as.array.n; i++) scan_expr(n->as.array.elems[i], scope, captured, depth, selfn);
            break;
        case AST_MAP:
            for (size_t i = 0; i < n->as.map.n; i++) {
                /* 裸标识符键是字符串键（{k: 1}），不参与捕获分析 */
                if (n->as.map.entries[i].key->kind != AST_IDENT) {
                    scan_expr(n->as.map.entries[i].key, scope, captured, depth, selfn);
                }
                scan_expr(n->as.map.entries[i].value, scope, captured, depth, selfn);
            }
            break;
        case AST_UNARY:
            scan_expr(n->as.unary.operand, scope, captured, depth, selfn);
            break;
        case AST_BINARY:
            scan_expr(n->as.binary.left, scope, captured, depth, selfn);
            scan_expr(n->as.binary.right, scope, captured, depth, selfn);
            break;
        case AST_TERNARY:
            scan_expr(n->as.ternary.cond, scope, captured, depth, selfn);
            scan_expr(n->as.ternary.then_, scope, captured, depth, selfn);
            scan_expr(n->as.ternary.else_, scope, captured, depth, selfn);
            break;
        case AST_CALL:
            scan_expr(n->as.call.callee, scope, captured, depth, selfn);
            for (size_t i = 0; i < n->as.call.n_args; i++) scan_expr(n->as.call.args[i].expr, scope, captured, depth, selfn);
            break;
        case AST_INDEX:
            scan_expr(n->as.index.obj, scope, captured, depth, selfn);
            scan_expr(n->as.index.index, scope, captured, depth, selfn);
            break;
        case AST_MEMBER:
            scan_expr(n->as.member.obj, scope, captured, depth, selfn);
            /* super.method 需要 self 被 lambda 捕获（若当前作用域无 self） */
            if (n->as.member.obj->kind == AST_IDENT &&
                strcmp(n->as.member.obj->as.ident.name, "super") == 0 && selfn &&
                !name_set_has(scope, selfn)) {
                name_set_add(captured, selfn);
            }
            break;
        case AST_LAMBDA: {
            /* 新 lambda 的 scope 从零开始（只含其参数）：其体引用的外层名是捕获候选 */
            NameSet lscope = {0};
            for (size_t i = 0; i < n->as.lambda.n_params; i++) {
                if (n->as.lambda.params[i].name) name_set_add(&lscope, n->as.lambda.params[i].name);
            }
            scan_body(n->as.lambda.body, &lscope, captured, selfn);
            name_set_free(&lscope);
            break;
        }
        case AST_THIS_FUNC:
            break;
        case AST_YIELD:
            if (n->as.yield_.expr) scan_expr(n->as.yield_.expr, scope, captured, depth, selfn);
            break;
        case AST_DEFER: {
            NameSet lscope = {0};
            scan_body(n->as.defer_.block, &lscope, captured, selfn);
            name_set_free(&lscope);
            break;
        }
        case AST_TRY:
            scan_expr(n->as.try_.expr, scope, captured, depth, selfn);
            break;
        case AST_OPTIONAL:
            scan_expr(n->as.optional.obj, scope, captured, depth, selfn);
            for (size_t i = 0; i < n->as.optional.n_args; i++)
                scan_expr(n->as.optional.args[i].expr, scope, captured, depth, selfn);
            break;
        case AST_CATCH: {
            scan_expr(n->as.catch_.expr, scope, captured, depth, selfn);
            const CatchHandler *h = &n->as.catch_.handler;
            if (h->kind == CATCH_BLOCK) {
                size_t mark = scope->n;
                if (h->error_name) name_set_add(scope, h->error_name);
                scan_stmt(h->fallback, scope, captured, depth + 1, selfn);
                scope->n = mark;
            } else if (h->kind == CATCH_FALLBACK) {
                scan_expr(h->fallback, scope, captured, depth, selfn);
            } else { /* CATCH_MATCH */
                for (size_t i = 0; i < h->n_arms; i++) {
                    size_t mark = scope->n;
                    for (size_t j = 0; j < h->arms[i].n_payload; j++) {
                        name_set_add(scope, h->arms[i].payload[j]);
                    }
                    const AstNode *av = h->arms[i].value;
                    if (av && av->kind == AST_BLOCK) scan_stmt(av, scope, captured, depth + 1, selfn);
                    else scan_expr(av, scope, captured, depth, selfn);
                    scope->n = mark;
                }
            }
            break;
        }
        case AST_MATCH: {
            scan_expr(n->as.match_.expr, scope, captured, depth, selfn);
            for (size_t i = 0; i < n->as.match_.n_arms; i++) {
                const AstMatchArm *arm = &n->as.match_.arms[i];
                size_t mark = scope->n;
                if (arm->pattern == ARM_BIND && arm->bind) name_set_add(scope, arm->bind);
                for (size_t j = 0; j < arm->n_payload; j++) name_set_add(scope, arm->payload[j]);
                const AstNode *av = arm->value;
                if (av && av->kind == AST_BLOCK) scan_stmt(av, scope, captured, depth + 1, selfn);
                else scan_expr(av, scope, captured, depth, selfn);
                scope->n = mark;
            }
            break;
        }
        case AST_IF:
        case AST_WHILE:
        case AST_FOR_IN:
        case AST_BLOCK_EXPR:
            scan_stmt(n, scope, captured, depth, selfn);
            break;
        case AST_FUNC_DECL:
            break; /* 匿名 func 不捕获 */
        default:
            break;
    }
}

/* 收集函数体中被嵌套 lambda 捕获的自由名。start_depth: 0=程序顶层, 1=函数体块。
   selfn：当前方法的 self 参数名（super 需其被捕获），非方法为 NULL。 */
static void collect_captured_body(const AstNode *body, const AstParam *params, size_t n_params,
                                  int start_depth, NameSet *out, const char *selfn) {
    NameSet scope = {0};
    for (size_t i = 0; i < n_params; i++) {
        if (params[i].name) name_set_add(&scope, params[i].name);
    }
    if (body && body->kind == AST_BLOCK) {
        for (size_t i = 0; i < body->as.block.n; i++) {
            scan_stmt(body->as.block.stmts[i], &scope, out, start_depth, selfn);
        }
    } else {
        scan_expr(body, &scope, out, start_depth, selfn);
    }
    name_set_free(&scope);
}

static void begin_scope(Codegen *c) {
    c->scope_depth++;
}

/* 所有 reg_top 回退都经此：不得低于 cap_floor（被捕获局部槽保留到帧返回，
   否则开放捕获值会指向被复用的寄存器槽）。 */
static void restore_regs(Codegen *c, int mark) {
    if (mark < c->cap_floor) mark = c->cap_floor;
    c->reg_top = mark;
}

static void end_scope(Codegen *c) {
    c->scope_depth--;
    while (c->n_locals > 0 && c->locals[c->n_locals - 1].depth > c->scope_depth) {
        const Local *l = &c->locals[c->n_locals - 1];
        if (l->is_captured && l->slot + 1 > c->cap_floor) c->cap_floor = l->slot + 1;
        c->n_locals--;
        c->reg_top--;
    }
    if (c->reg_top < c->cap_floor) c->reg_top = c->cap_floor;
}

/* ---------- 数字字面量解析 ---------- */

/* 把 base 进制数字串按 double 累加（strtoll 溢出后的兜底，如超大 0x/0b/0o 字面量）。 */
static double parse_radix_double(const char *s, int base) {
    double d = 0;
    for (; *s; s++) {
        const char ch = *s;
        int v;
        if (ch >= '0' && ch <= '9') v = ch - '0';
        else if (ch >= 'a' && ch <= 'f') v = ch - 'a' + 10;
        else if (ch >= 'A' && ch <= 'F') v = ch - 'A' + 10;
        else break;
        if (v >= base) break;
        d = d * base + v;
    }
    return d;
}

static Value parse_number_text(Codegen *c, const char *text, size_t len) {
    (void)c;
    char *tmp = malloc(len + 1);
    assert(tmp != NULL);
    size_t out = 0;
    /* 词法器接受两种分隔符：'_'（历史）与 '\''（lexer.c:145-163 的 scan_digits 规范形式）。
       **两者都必须剥掉**——只剥 '_' 会让 `1'000` 变成 `1`（stroll 在 ''' 处停下，
       而旧代码不检查 end），静默产生错误常量。 */
    for (size_t i = 0; i < len; i++) {
        if (text[i] != '_' && text[i] != '\'') tmp[out++] = text[i];
    }
    tmp[out] = '\0';

    /* 判定进制：0x/0o/0b 前缀。注意 hex 的数字串含 a-f，其中的 e/E 不是指数！ */
    int base = 10;
    const char *digits = tmp;
    if (out >= 2 && tmp[0] == '0' && (tmp[1] == 'x' || tmp[1] == 'X')) {
        base = 16;
        digits = tmp + 2;
    } else if (out >= 2 && tmp[0] == '0' && (tmp[1] == 'o' || tmp[1] == 'O')) {
        base = 8;
        digits = tmp + 2;
    } else if (out >= 2 && tmp[0] == '0' && (tmp[1] == 'b' || tmp[1] == 'B')) {
        base = 2;
        digits = tmp + 2;
    }

    /* float 触发 1：含小数部分（十进制/任何进制，mica 仅十进制支持小数）。 */
    if (strchr(tmp, '.')) {
        errno = 0;
        char *fend = NULL;
        double d = strtod(tmp, &fend);
        if (fend == tmp || (fend && *fend != '\0')) {
            /* 非法数字字面量：旧实现忽略 end，把 "1abc" 之类静默解析成 1 */
            free(tmp);
            error(c, NULL, "invalid number literal '%s'", text);
            return val_int(0);
        }
        free(tmp);
        return val_float(d);
    }
    /* float 触发 2：仅十进制把 e/E 当科学计数指数（hex 的 e/E 是数字位）。 */
    if (base == 10 && (strchr(tmp, 'e') || strchr(tmp, 'E'))) {
        char *fend = NULL;
        double d = strtod(tmp, &fend);
        if (fend == tmp || (fend && *fend != '\0')) {
            free(tmp);
            error(c, NULL, "invalid number literal '%s'", text);
            return val_int(0);
        }
        free(tmp);
        return val_float(d);
    }

    char *end;
    errno = 0;
    if (base == 2) {
        /* 二进制：手写 int 累加，溢出→double（strtoll 不认识 0b 前缀）。 */
        unsigned long long acc = 0;
        bool overflow = false;
        for (const char *s = digits; *s; s++) {
            if (*s != '0' && *s != '1') break;
            if (acc > (unsigned long long)INT64_MAX / 2) overflow = true;
            acc = acc * 2 + (unsigned)(*s - '0');
        }
        const double dbig = parse_radix_double(digits, 2);
        free(tmp);
        if (overflow) return val_float(dbig);
        return val_float((double)(int64_t)acc);
    }

    /* 8/10/16 进制：strtoll 超 int64（ERANGE）才转 double。end 须消费完整串
       （旧实现忽略 end：`0x1g` 之类静默变成 1）。 */
    errno = 0;
    long long v = strtoll(digits, &end, base);
    if (end == digits || (end && *end != '\0')) {
        free(tmp);
        error(c, NULL, "invalid number literal '%s'", text);
        return val_int(0);
    }
    if (errno == ERANGE) {
        double d = (base == 10) ? strtod(digits, NULL) : parse_radix_double(digits, base);
        free(tmp);
        return val_float(d);
    }
    free(tmp);
    return val_float((double)(int64_t)v);
}

/* 计算枚举成员的无 payload 整数值（仿 codegen_enum_build 递增逻辑：从 0 起，
   带 payload 成员不占位）。member 必须是 set 的无 payload 成员；显式值仅支持
   数字字面量。无法静态求值返回 false。 */
static bool enum_member_int_value(Codegen *c, const EnumSetInfo *set,
                                  const AstEnumMember *member, int64_t *out) {
    int64_t prev = -1;
    for (size_t i = 0; i < set->n_members; i++) {
        const AstEnumMember *m = &set->members[i];
        if (m->n_payload > 0) continue;
        if (m->value) {
            if (m->value->kind != AST_NUMBER) return false;
            Value v = parse_number_text(c, m->value->as.number.text, m->value->as.number.len);
            int64_t ival;
            if (!val_is_num(v) || !float_as_int(val_as_float(v), &ival)) return false;
            prev = ival;
        } else {
            prev += 1;
        }
        if (m == member) {
            *out = prev;
            return true;
        }
    }
    return false;
}

/* ---------- 表达式 ---------- */

static void codegen_expr(Codegen *c, const AstNode *n, int dest);
static void codegen_stmt(Codegen *c, const AstNode *n);
static void codegen_block_stmt(Codegen *c, const AstNode *block);
static void codegen_match_arms(Codegen *c, const AstMatchArm *arms, size_t n_arms,
                               int value_reg, int dest, int line, bool match_error,
                               bool no_match_errors);
static void codegen_if(Codegen *c, const AstNode *n, int value_dest);
static void codegen_while(Codegen *c, const AstNode *n, int value_dest);
static void codegen_for(Codegen *c, const AstNode *n, int value_dest);
static ObjClosure *codegen_func_value(Codegen *c, const AstNode *func);
static void codegen_lambda(Codegen *c, const AstNode *n, int dest);
static void codegen_coroutine(Codegen *c, const AstNode *body, ObjClosure *f, int dest, int line);
static void codegen_yield(Codegen *c, const AstNode *n, int dest);
static void codegen_defer(Codegen *c, const AstNode *n);
static bool body_has_yield(const AstNode *body);

static void codegen_binary(Codegen *c, const AstNode *n, int dest);
static void codegen_logical(Codegen *c, const AstNode *n, int dest);
static void codegen_null_coalesce(Codegen *c, const AstNode *n, int dest);
static void codegen_ternary(Codegen *c, const AstNode *n, int dest);
static void codegen_is(Codegen *c, const AstNode *n, int dest);
static void codegen_match(Codegen *c, const AstNode *n, int dest);
static void codegen_throw(Codegen *c, const AstNode *n);
static void codegen_try_block(Codegen *c, const AstNode *n);
static void codegen_do_catch(Codegen *c, const AstNode *n);
static void codegen_optional(Codegen *c, const AstNode *n, int dest);
static void codegen_call(Codegen *c, const AstNode *n, int dest, uint16_t mode);
static void codegen_string(Codegen *c, const AstNode *n, int dest);
static void codegen_ident(Codegen *c, const AstNode *n, int dest);
static void codegen_array(Codegen *c, const AstNode *n, int dest);
static void codegen_map(Codegen *c, const AstNode *n, int dest);
static void codegen_index_get(Codegen *c, const AstNode *n, int dest);
static void codegen_member_get(Codegen *c, const AstNode *n, int dest);
static bool codegen_error_case_construct(Codegen *c, const AstNode *call, int dest, uint16_t mode);
static bool codegen_enum_case_construct(Codegen *c, const AstNode *call, int dest, uint16_t mode);

static OpCode binop_to_opcode(int tok) {
    switch (tok) {
        case TOK_PLUS:
        case TOK_PLUS_ASSIGN: return OP_ADD;
        case TOK_MINUS:
        case TOK_MINUS_ASSIGN: return OP_SUB;
        case TOK_STAR:
        case TOK_STAR_ASSIGN: return OP_MUL;
        case TOK_SLASH:
        case TOK_SLASH_ASSIGN: return OP_DIV;
        case TOK_SLASH_SLASH:
        case TOK_SLASH_SLASH_ASSIGN: return OP_IDIV;
        case TOK_PERCENT:
        case TOK_PERCENT_ASSIGN: return OP_MOD;
        case TOK_STAR_STAR:
        case TOK_STAR_STAR_ASSIGN: return OP_POW;
        case TOK_EQ: return OP_EQ;
        case TOK_BANG_EQ: return OP_NE;
        case TOK_LT: return OP_LT;
        case TOK_LE: return OP_LE;
        case TOK_GT: return OP_LT;   /* 反向操作数：a > b ≡ b < a（LuaJIT 思想，无独立 GT） */
        case TOK_GE: return OP_LE;   /* a >= b ≡ b <= a */
        default: return OP_COUNT; /* 非法 */
    }
}

/* 比较需交换操作数的 token（GT/GE 用 LT/LE 反向表达）：调用方发射前交换 t0/t1。 */
static bool binop_swap_operands(int tok) {
    return tok == TOK_GT || tok == TOK_GE;
}

static void codegen_unary(Codegen *c, const AstNode *n, int dest) {
    /* 常量折叠：-数字字面量 直接装载负值（17 位内走 MOVI 单条，省 OP_NEG），
       也避免大整数先入常量池再取负。 */
    if (n->as.unary.op == TOK_MINUS && n->as.unary.operand->kind == AST_NUMBER) {
        Value v = parse_number_text(c, n->as.unary.operand->as.number.text,
                                    n->as.unary.operand->as.number.len);
        if (val_is_num(v)) {
            int64_t iv;
            if (float_as_int(val_as_float(v), &iv) && iv != INT64_MIN) { /* 整数 → MOVI 单条 */
                emit_int_const(c, dest, -iv, n->line);
                return;
            }
            /* float 或超 int64：KNUM 装载负值（-2^63 等由 double 表示） */
            emit_const_op(c, dest, val_float(-val_as_float(v)), n->line);
            return;
        }
    }
    int mark = c->reg_top;
    if (n->as.unary.op == TOK_PLUS) {
        /* 一元 +：无操作，直接求值操作数（既有 bug：原归入 OP_NOT） */
        codegen_expr(c, n->as.unary.operand, dest);
        restore_regs(c, mark);
        return;
    }
    OpCode op = (n->as.unary.op == TOK_MINUS) ? OP_NEG : OP_NOT;
    int src;
    if (expr_plain_reg(c, n->as.unary.operand, &src)) {
        emit(c, op, dest, src, 0, n->line);
    } else {
        int t = alloc_temp(c);
        codegen_expr(c, n->as.unary.operand, t);
        emit(c, op, dest, t, 0, n->line);
        restore_regs(c, mark);
    }
}

/* 阶段 B：表达式是否为可折叠常量字面量（数字/字符串/bool/null）——折叠算术
   （ADDVN/ADDNV 等）把操作数嵌入常量池下标，循环里 `s = s + 1` 从 2 条变 1 条。
   多段插值字符串（含 ${}）不是字面量，不折叠。 */
/* 表达式是否"可能调用用户代码"（call / 运算符重载 / 属性 getter 都可能通过闭包改写被
   捕获的局部或全局变量）。用于决定复合赋值能否在求 RHS **之前**把目标读进寄存器：
   若 RHS 可能改写目标（如 `x += f()` 且 f 改写 x），必须先读；否则可直接复用目标槽。
   保守方向：拿不准一律返回 true（多一次 MOV，换正确性）。 */
static bool expr_may_call(const AstNode *n) {
    if (!n) return true; /* 保守 */
    switch (n->kind) {
        case AST_CALL:        /* 调用（含方法调用、构造、运算符重载分发） */
        case AST_BLOCK_EXPR:  /* get { ... } 块表达式：块内可含语句 */
        case AST_TRY:
        case AST_CATCH:
        case AST_DO_CATCH:
        case AST_MATCH:
        case AST_YIELD:
        case AST_VARARGS:
            return true;
        case AST_BINARY:
            return true; /* 可能是运算符重载（调用实例方法） */
        case AST_UNARY:
            return true; /* 可能 __neg__/not 等 */
        case AST_TERNARY:
            return expr_may_call(n->as.ternary.cond) || expr_may_call(n->as.ternary.then_) ||
                   expr_may_call(n->as.ternary.else_);
        case AST_INDEX:
            return true; /* 可能 __index__ 重载 */
        case AST_MEMBER:
            return true; /* 可能属性 getter */
        case AST_ARRAY: {
            for (size_t i = 0; i < n->as.array.n; i++) {
                if (expr_may_call(n->as.array.elems[i])) return true;
            }
            return false;
        }
        case AST_LAMBDA:
        case AST_FUNC_DECL:
            return false; /* 函数值构造本身不执行函数体 */
        default:
            return false; /* 字面量 / 标识符 / null / true / false 等纯值 */
    }
}

static bool expr_const_value(Codegen *c, const AstNode *n, Value *out) {
    switch (n->kind) {
        case AST_NUMBER:
            *out = parse_number_text(c, n->as.number.text, n->as.number.len);
            return true;
        case AST_STRING: {
            if (n->as.str.n_parts == 1 && n->as.str.parts[0].is_literal) {
                const AstStrPart *p = &n->as.str.parts[0];
                *out = obj_string_val(obj_string_new_permanent(c->gc, p->text, p->len));
                return true;
            }
            if (n->as.str.n_parts == 0) {
                *out = obj_string_val(obj_string_new_permanent(c->gc, "", 0));
                return true;
            }
            return false; /* 插值 */
        }
        case AST_TRUE: *out = val_true(); return true;
        case AST_FALSE: *out = val_false(); return true;
        case AST_NULL: *out = val_null(); return true;
        default: return false;
    }
}

static void codegen_binary(Codegen *c, const AstNode *n, int dest) {
    if (n->as.binary.op == TOK_AND_AND || n->as.binary.op == TOK_OR_OR) {
        codegen_logical(c, n, dest);
        return;
    }
    if (n->as.binary.op == TOK_QUESTION_QUESTION) {
        codegen_null_coalesce(c, n, dest);
        return;
    }
    if (n->as.binary.op == TOK_DOT_DOT) {
        error(c, n, "range '..' is only allowed in for-in (v1)");
        return;
    }
    if (n->as.binary.op == TOK_IS) {
        codegen_is(c, n, dest);
        return;
    }
    OpCode op = binop_to_opcode(n->as.binary.op);
    if (op == OP_COUNT) {
        error(c, n, "unsupported binary operator in v1");
        return;
    }
    /* 阶段 B：折叠常量算术——右操作数是字面量 → VN（R[d] = R[l] op const），
       左操作数是字面量且交换律成立 → NV（R[d] = const op R[r]）。
       常量下标 ≤ 511（格式3：VN 的 C 字段 9 位；NV 的 B 字段 9 位；add_const 超限返回 -1）。 */
    {
        int imark = c->reg_top;
        Value icv;
        /* 立即数算术：右操作数是小整数常量（-128..127）→ ADDI/SUBI（免常量池 + 省一次类型检查）。
           超限回退 ADDVN（常量池）。 */
        if ((op == OP_ADD || op == OP_SUB) && expr_const_value(c, n->as.binary.right, &icv) &&
            val_is_num(icv)) {
            int64_t iv;
            if (float_as_int(val_as_float(icv), &iv) && iv >= -128 && iv <= 127) {
                int t0;
                if (!expr_plain_reg(c, n->as.binary.left, &t0)) {
                    t0 = alloc_temp(c);
                    codegen_expr(c, n->as.binary.left, t0);
                }
                emit(c, op == OP_ADD ? OP_ADDI : OP_SUBI, dest, t0, (int)(int8_t)iv, n->line);
                restore_regs(c, imark);
                return;
            }
        }
        OpCode vn_op = OP_COUNT, nv_op = OP_COUNT;
        switch (op) {
            case OP_ADD: vn_op = OP_ADDVN; nv_op = OP_ADDNV; break;
            case OP_SUB: vn_op = OP_SUBVN; break;
            case OP_MUL: vn_op = OP_MULVN; nv_op = OP_MULNV; break;
            case OP_DIV: vn_op = OP_DIVVN; break;
            case OP_IDIV: vn_op = OP_IDIVVN; break;
            case OP_MOD: vn_op = OP_MODVN; break;
            default: break;
        }
        if (vn_op != OP_COUNT) {
            int fmark = c->reg_top;
            Value cv;
            int k;
            /* VN/NV 只折叠数字常量（ADDVN 族读 num 池；字符串操作数回退寄存器路径走拼接） */
            if (expr_const_value(c, n->as.binary.right, &cv) && val_is_num(cv) &&
                (k = add_const(c, cv)) >= 0 && k < 512) {
                int t0;
                if (!expr_plain_reg(c, n->as.binary.left, &t0)) {
                    t0 = alloc_temp(c);
                    codegen_expr(c, n->as.binary.left, t0);
                }
                emit(c, vn_op, dest, t0, k, n->line);
                restore_regs(c, fmark);
                return;
            }
            if (nv_op != OP_COUNT && expr_const_value(c, n->as.binary.left, &cv) &&
                val_is_num(cv) && (k = add_const(c, cv)) >= 0 && k < 256) {
                /* ADDNV/MULNV：常量在 B 字段（8 位，≤255）；超限回退寄存器路径。 */
                int t1;
                if (!expr_plain_reg(c, n->as.binary.right, &t1)) {
                    t1 = alloc_temp(c);
                    codegen_expr(c, n->as.binary.right, t1);
                }
                emit(c, nv_op, dest, k, t1, n->line);
                restore_regs(c, fmark);
                return;
            }
            restore_regs(c, fmark);
        }
    }
    int mark = c->reg_top;
    int t0 = -1, t1 = -1;
    if (!expr_plain_reg(c, n->as.binary.left, &t0)) {
        t0 = alloc_temp(c);
        codegen_expr(c, n->as.binary.left, t0);
    }
    if (!expr_plain_reg(c, n->as.binary.right, &t1)) {
        t1 = alloc_temp(c);
        codegen_expr(c, n->as.binary.right, t1);
    }
    if (binop_swap_operands(n->as.binary.op)) { /* GT/GE → 交换操作数用 LT/LE */
        const int tmp = t0;
        t0 = t1;
        t1 = tmp;
    }
    emit(c, op, dest, t0, t1, n->line);
    restore_regs(c, mark);
}

static void codegen_logical(Codegen *c, const AstNode *n, int dest) {
    const bool is_and = (n->as.binary.op == TOK_AND_AND);
    int mark = c->reg_top;
    int t = alloc_temp(c);
    codegen_expr(c, n->as.binary.left, t);
    size_t j = proto_code_len(c->proto);
    emit_jump(c, is_and ? OP_JMP_IF_FALSE : OP_JMP_IF_TRUE, t, 0, n->line);
    codegen_expr(c, n->as.binary.right, t);
    patch_jump(c, j, proto_code_len(c->proto));
    emit(c, OP_MOV, dest, t, 0, n->line);
    restore_regs(c, mark);
}

/* a ?? b：a 非 null 取 a，否则 b（短路，只检查 null——false/0/"" 都算非 null）。 */
static void codegen_null_coalesce(Codegen *c, const AstNode *n, int dest) {
    int mark = c->reg_top;
    codegen_expr(c, n->as.binary.left, dest); /* dest = a */
    int nreg = alloc_temp(c);
    emit_const_op(c, nreg, val_null(), n->line);
    emit(c, OP_ISNE, dest, nreg, 0, n->line); /* dest != null → 跳过下一条 JMP */
    size_t gb = proto_code_len(c->proto);
    emit(c, OP_JMP, 0, 0, 0, n->line);        /* dest == null → 求 b */
    size_t done = proto_code_len(c->proto);
    emit(c, OP_JMP, 0, 0, 0, n->line);        /* dest != null → 跳到结束 */
    patch_jump(c, gb, (int)proto_code_len(c->proto));
    codegen_expr(c, n->as.binary.right, dest);
    patch_jump(c, done, (int)proto_code_len(c->proto));
    restore_regs(c, mark);
}

static void codegen_ternary(Codegen *c, const AstNode *n, int dest) {
    int mark = c->reg_top;
    int t = alloc_temp(c);
    codegen_expr(c, n->as.ternary.cond, t);
    size_t jf = proto_code_len(c->proto);
    emit_jump(c, OP_JMP_IF_FALSE, t, 0, n->line);
    codegen_expr(c, n->as.ternary.then_, dest);
    size_t je = proto_code_len(c->proto);
    emit_jump(c, OP_JMP, 0, 0, n->line);
    patch_jump(c, jf, proto_code_len(c->proto));
    codegen_expr(c, n->as.ternary.else_, dest);
    patch_jump(c, je, proto_code_len(c->proto));
    restore_regs(c, mark);
}

/* 调用目标是否为 `System.Builtin.<name>`（编译器开洞的内置命名空间）。 */
static bool is_builtin_call(const AstNode *callee, const char **out_name) {
    if (!callee || callee->kind != AST_MEMBER) return false;
    const AstNode *mid = callee->as.member.obj;
    if (!mid || mid->kind != AST_MEMBER) return false;
    const AstNode *root = mid->as.member.obj;
    if (!root || root->kind != AST_IDENT) return false;
    if (strcmp(root->as.ident.name, "System") != 0) return false;
    if (strcmp(mid->as.member.name, "Builtin") != 0) return false;
    if (out_name) *out_name = callee->as.member.name;
    return true;
}

/* ---------- 命名参数：静态签名解析与重排 ---------- */

/* 表达式是否 `ClassName(...)`（构造调用）？返回类名，否则 NULL。 */
static const char *callee_class_name(Codegen *c, const AstNode *expr) {
    if (expr->kind != AST_CALL || expr->as.call.callee->kind != AST_IDENT) return NULL;
    const char *name = expr->as.call.callee->as.ident.name;
    if (find_class_entry(c, name)) return name;
    return NULL;
}

/* 沿类继承链找方法/init（want_static 区分静态/实例）。返回方法 AST 或 NULL。 */
static const AstNode *find_class_method_inherit(Codegen *c, const AstNode *class_ast,
                                                const char *name, bool want_static) {
    for (const AstNode *ca = class_ast; ca;) {
        for (size_t i = 0; i < ca->as.class_.n_members; i++) {
            const AstNode *m = ca->as.class_.members[i];
            if (m->kind == AST_FUNC_DECL && m->as.func.name &&
                strcmp(m->as.func.name, name) == 0 && m->as.func.is_static == want_static &&
                !m->as.func.is_property) {
                return m;
            }
        }
        if (!ca->as.class_.parent) break;
        ClassEntry *pe = find_class_entry(c, ca->as.class_.parent);
        ca = pe ? pe->ast : NULL;
    }
    return NULL;
}

/* 被调函数签名（命名参数重排用），镜像 sema 的 resolve_callee。 */
typedef struct {
    const AstParam *params;
    size_t n_params;
    bool skip_self;   /* 方法/构造：self 自动绑定，首参数跳过匹配 */
    bool has_varargs;
    const char *desc;
    bool found;
} CalleeSig;

static void codegen_resolve_callee(Codegen *c, const AstNode *callee, CalleeSig *out) {
    memset(out, 0, sizeof(*out));
    if (callee->kind == AST_IDENT) {
        const char *name = callee->as.ident.name;
        const FuncSig *fs = find_func_sig(c, name);
        if (fs) {
            out->params = fs->params;
            out->n_params = fs->n_params;
            out->has_varargs = fs->has_varargs;
            out->desc = fs->name;
            out->found = true;
            return;
        }
        /* 类构造：ClassName(args) → init(self, ...) */
        ClassEntry *ce = find_class_entry(c, name);
        if (ce && ce->ast) {
            const AstNode *init = find_class_method_inherit(c, ce->ast, "init", false);
            if (init) {
                out->params = init->as.func.params;
                out->n_params = init->as.func.n_params;
                out->skip_self = true;
                out->has_varargs = init->as.func.has_varargs;
                out->desc = name;
                out->found = true;
            }
        }
        return;
    }
    if (callee->kind == AST_MEMBER && callee->as.member.obj->kind == AST_IDENT) {
        const char *base = callee->as.member.obj->as.ident.name;
        const char *mname = callee->as.member.name;
        /* 静态方法：Class.m(...) */
        ClassEntry *ce = find_class_entry(c, base);
        if (ce && ce->ast) {
            const AstNode *m = find_class_method_inherit(c, ce->ast, mname, true);
            if (m) {
                out->params = m->as.func.params;
                out->n_params = m->as.func.n_params;
                out->has_varargs = m->as.func.has_varargs;
                out->desc = mname;
                out->found = true;
                return;
            }
        }
        /* 实例方法：obj.m(...)，obj 经 `var obj = ClassName()` 推断 */
        const char *cls = find_var_class(c, base);
        if (cls) {
            ClassEntry *ce2 = find_class_entry(c, cls);
            if (ce2 && ce2->ast) {
                const AstNode *m = find_class_method_inherit(c, ce2->ast, mname, false);
                if (m) {
                    out->params = m->as.func.params;
                    out->n_params = m->as.func.n_params;
                    out->skip_self = true;
                    out->has_varargs = m->as.func.has_varargs;
                    out->desc = mname;
                    out->found = true;
                }
            }
        }
    }
}

/* 编译调用实参到连续槽。统一调用约定（2026-08-15）：slot0=函数值、slot1=self（方法=receiver、
   其余=null，已由 codegen_call_callee 设置）、实参从 func_reg+2 排。argc 恒含 self（slot1）。
   返回 0=成功；出错返回 1（c->has_error 已置）。 */
static int codegen_emit_args(Codegen *c, const AstNode *call, int func_reg, uint16_t *out_argc) {
    const int arg_start = func_reg + 2;
    (void)arg_start; /* 仅断言使用；Release(NDEBUG) 下防 C4189 */
    bool has_named = false;
    for (size_t i = 0; i < call->as.call.n_args; i++) {
        if (call->as.call.args[i].named) { has_named = true; break; }
    }
    if (!has_named) {
        *out_argc = 1; /* self 槽（slot1） */
        for (size_t i = 0; i < call->as.call.n_args; i++) {
            int slot = alloc_temp(c);
            codegen_expr(c, call->as.call.args[i].expr, slot);
            (*out_argc)++;
        }
        return 0;
    }

    /* 命名参数：静态解析被调函数签名，按参数名重排为位置参数 */
    CalleeSig sig;
    codegen_resolve_callee(c, call->as.call.callee, &sig);
    if (!sig.found) {
        error(c, call, "named arguments require a statically resolvable function or method");
        return 1;
    }
    if (sig.has_varargs) {
        error(c, call, "named arguments are not supported for vararg functions");
        return 1;
    }
    const size_t first = (sig.skip_self && sig.n_params > 0) ? 1 : 0;
    const size_t n_slots = sig.n_params - first;
    int *slots = malloc((n_slots ? n_slots : 1) * sizeof(int));
    bool *filled = calloc(n_slots ? n_slots : 1, sizeof(bool));
    assert(slots != NULL && filled != NULL);
    for (size_t i = 0; i < n_slots; i++) {
        slots[i] = alloc_temp(c);
        assert(slots[i] == arg_start + (int)i); /* 实参槽须紧接（self 之后的）函数槽 */
    }

    /* 位置参数按顺序填最低未占用槽 */
    size_t pos_cursor = 0;
    for (size_t i = 0; i < call->as.call.n_args; i++) {
        const AstArg *a = &call->as.call.args[i];
        if (a->named) continue;
        if (pos_cursor >= n_slots) {
            error(c, call, "too many positional arguments for %s", sig.desc);
            free(slots); free(filled); return 1;
        }
        if (filled[pos_cursor]) {
            error(c, call, "argument '%s' provided more than once",
                  sig.params[pos_cursor + first].name ? sig.params[pos_cursor + first].name : "?");
            free(slots); free(filled); return 1;
        }
        codegen_expr(c, a->expr, slots[pos_cursor]);
        filled[pos_cursor] = true;
        pos_cursor++;
    }
    /* 命名参数填到名字对应槽 */
    for (size_t i = 0; i < call->as.call.n_args; i++) {
        const AstArg *a = &call->as.call.args[i];
        if (!a->named) continue;
        size_t idx = SIZE_MAX;
        for (size_t j = first; j < sig.n_params; j++) {
            if (sig.params[j].name && strcmp(sig.params[j].name, a->name) == 0) {
                idx = j - first;
                break;
            }
        }
        if (idx == SIZE_MAX) {
            error(c, call, "unknown named argument '%s' for %s", a->name, sig.desc);
            free(slots); free(filled); return 1;
        }
        if (filled[idx]) {
            error(c, call, "argument '%s' provided more than once", a->name);
            free(slots); free(filled); return 1;
        }
        codegen_expr(c, a->expr, slots[idx]);
        filled[idx] = true;
    }
    /* 未填位置：有默认参数 → 填 null 触发默认 prologue；否则缺参报错 */
    for (size_t i = 0; i < n_slots; i++) {
        if (!filled[i]) {
            const AstParam *p = &sig.params[i + first];
            if (p->has_default) {
                emit(c, OP_NULL, slots[i], 0, 0, call->line);
            } else {
                error(c, call, "missing required argument '%s' for %s",
                      p->name ? p->name : "?", sig.desc);
                free(slots); free(filled); return 1;
            }
        }
    }
    free(slots);
    free(filled);
    *out_argc = (uint16_t)(n_slots + 1); /* 恒含 self 槽 */
    return 0;
}

/* 记录整体 import 的模块名：全名 + 末段短名（import System.Math 后 Math.sqrt 也可用）。
   命名空间成员函数调用据此不注入 receiver。 */
static void add_imported_module(Codegen *c, const char *name) {
    if (!name) return;
    name_set_add(&c->imported_modules, name);
    const char *dot = strrchr(name, '.');
    if (dot && dot[1]) {
        name_set_add(&c->imported_modules, dot + 1);
        /* 方案 A：模块别名 → 全限定名（Math → System.Math），压平 Math.member 引用 */
        import_map_add(c, dot + 1, name);
    }
}

/* 登记一个 import 的全部信息（整体模块别名 + 选择性成员 → 全限定名）。
   预扫描（函数体编译前）与主 pass 都用它——压平映射必须在编译任何函数体前就绪。 */
static void codegen_import_scan(Codegen *c, const AstNode *n) {
    if (n->as.import.name) add_imported_module(c, n->as.import.name);
    for (size_t i = 0; i < n->as.import.n_names; i++) {
        const char *member = n->as.import.names[i];
        char buf[512];
        snprintf(buf, sizeof(buf), "%s.%s", n->as.import.name, member);
        import_map_add(c, member, buf); /* sqrt → System.Math.sqrt（不再建裸全局） */
    }
}

/* 整体 import 的模块名（命名空间成员函数调用不注入 receiver）。name 若是某模块名的
   首段（System 之于 System.Runtime）也算命中。 */
static bool is_imported_module(Codegen *c, const char *name) {
    if (strcmp(name, "System") == 0) return true; /* System.* 内置命名空间（含 System.Builtin 开洞），非实例 */
    const size_t nl = strlen(name);
    for (Codegen *cc = c; cc; cc = cc->enclosing) {
        for (size_t i = 0; i < cc->imported_modules.n; i++) {
            const char *mod = cc->imported_modules.names[i];
            if (strcmp(mod, name) == 0) return true;
            if (strncmp(mod, name, nl) == 0 && mod[nl] == '.') return true;
        }
    }
    return false;
}

/* 求值调用 callee 到 func_reg（slot0=函数值），并设置 slot1=self（统一约定 2026-08-15：
   slot0=func、slot1=self、slot2+=实参）。实例成员/协程/super 把 receiver 注入 slot1；
   静态方法/模块函数/普通函数/字段闭包调用把 slot1 置 null。字段闭包（self.apply(x)）无需
   特殊区分——receiver 恒放 slot1，lambda 忽略它、方法读它当 self。 */
static void codegen_call_callee(Codegen *c, const AstNode *callee, int func_reg, int line) {
    if (callee->kind == AST_MEMBER) {
        const AstNode *obj = callee->as.member.obj;
        if (obj->kind == AST_IDENT && strcmp(obj->as.ident.name, "super") == 0) {
            /* super.m：从父类取方法（非虚分派），self 作 receiver 注入 func_reg+1 */
            if (!c->cur_class || !c->cur_class->superclass) {
                error(c, callee, "super used outside a method with a superclass");
                return;
            }
            int pclass = alloc_temp(c); /* = func_reg+1；先放父类常量，GET_METHOD 后复用为 receiver */
            emit(c, OP_CONST, pclass, add_const(c, obj_class_val(c->cur_class->superclass)), 0, line);
            int k = intern_name(c, callee->as.member.name);
            emit(c, OP_GET_METHOD, func_reg, k, pclass, line);
            if (c->self_name) {
                int slot = resolve_local(c, c->self_name);
                if (slot >= 0) emit(c, OP_MOV, pclass, slot, 0, line);
                else {
                    int cap = resolve_capval(c, c->self_name);
                    if (cap >= 0) emit(c, OP_GET_CAPVAL, pclass, cap, 1, line);
                    else emit(c, OP_LOAD_GLOBAL, pclass, global_name(c, c->self_name), 0, line);
                }
            } else {
                error(c, callee, "self is not available for super");
            }
            return;
        }
        /* 判定接收者是否为实例：取最左标识符（self.player.hit 的最左是 self，
           Shapes.Rect.unit / Math.square 的最左是模块/命名空间名）。
           类/模块命名空间成员函数 slot1=null；实例/局部/动态表达式 slot1=receiver。 */
        const AstNode *base = obj;
        while (base->kind == AST_MEMBER) base = base->as.member.obj;
        bool is_instance = false;
        if (base->kind == AST_IDENT) {
            const char *bname = base->as.ident.name;
            if (find_class_entry(c, bname)) {
                is_instance = false; /* 类命名空间（Class.m） */
            } else if (is_imported_module(c, bname)) {
                is_instance = false; /* 模块命名空间（Math.square / Shapes.Rect.unit） */
            } else {
                is_instance = true;  /* 局部/参数/self/全局实例/map */
            }
        } else {
            is_instance = true;      /* 动态表达式（getObj().m） */
        }
        if (is_instance) {
            /* 实例成员（方法/内置/协程/字段闭包统一）：obj 求到 func_reg+1（self 槽），
               GET_MEMBER 取成员到 func_reg。字段闭包（lambda）忽略 self 槽，方法读它。 */
            int obj_reg = alloc_temp(c); /* = func_reg+1 */
            codegen_expr(c, obj, obj_reg);
            int k = intern_name(c, callee->as.member.name);
            emit(c, OP_GET_MEMBER, func_reg, k, obj_reg, line);
            return;
        }
        /* 类/模块命名空间成员（静态方法/模块函数）：无 receiver → slot1 = null */
        codegen_expr(c, callee, func_reg);
        alloc_temp(c); /* func_reg+1 = self 槽（占位，null） */
        emit(c, OP_NULL, func_reg + 1, 0, 0, line);
        return;
    }
    /* 普通表达式调用（标识符/闭包变量/类名构造）：slot1 = null self */
    codegen_expr(c, callee, func_reg);
    alloc_temp(c); /* func_reg+1 = self 槽（占位，null） */
    emit(c, OP_NULL, func_reg + 1, 0, 0, line);
}

/* 发射一次调用：编译期确认非 throws 的已知函数 → FASTCALL（不检查）；
   否则 CALL（调用点被 try/catch 保护时 c 带 RESULT_PROTECT 位——未捕获错误退出检查据此豁免）。
   throws 标注已可选（2026-08-11）：未保护调用返回错误值 → 运行期报错退出。 */
static void codegen_emit_call(Codegen *c, const AstNode *callee, int func_reg, uint16_t argc,
                              uint16_t mode, int line) {
    /* 结果数上限：结果模式复用 c 字段的低 7 位（RESULT_MASK=0x7f），1..126 表示固定个数，
       127=RESULT_VARARGS。≥128 会与 RESULT_PROTECT(0x80) 冲突 → 静默截断成 0 个结果
       （实测 128 个返回值时前面变 null、其余变 0）。显式报错。 */
    if (mode != RESULT_VARARGS && mode > 126) {
        error(c, NULL, "too many assignment targets: %u (limit 126)", (unsigned)mode);
        return;
    }
    bool known_non_throws = false;
    if (callee->kind == AST_IDENT) {
        const FuncSig *sig = find_func_sig(c, callee->as.ident.name);
        if (sig && !sig->is_throws) known_non_throws = true;
    }
    const bool protected = c->protect_depth > 0;
    /* throws 标注可选后，不标 throws 的函数也可能 throw——受保护调用（try/catch/do-catch 内）
       必须用 OP_CALL + RESULT_PROTECT（未保护错误退出检查豁免），不能用 FASTCALL。 */
    if (known_non_throws && !protected) {
        emit(c, OP_FASTCALL, func_reg, argc, mode, line);
    } else {
        emit(c, OP_CALL, func_reg, argc, (int)mode | (protected ? RESULT_PROTECT : 0), line);
    }
}

static void codegen_call(Codegen *c, const AstNode *n, int dest, uint16_t mode) {
    if (codegen_error_case_construct(c, n, dest, mode)) return;
    if (codegen_enum_case_construct(c, n, dest, mode)) return;
    /* 编译期拦截：System.Builtin 仅标准库（System.* 模块）可调用，用户调用直接报错 */
    if (is_builtin_call(n->as.call.callee, NULL) &&
        !(c->module_prefix && strncmp(c->module_prefix, "System.", 7) == 0)) {
        error(c, n, "System.Builtin is internal: can only be called inside the standard library");
        return;
    }
    int mark = c->reg_top;
    int func_reg = alloc_temp(c);
    codegen_call_callee(c, n->as.call.callee, func_reg, n->line);
    uint16_t argc = 0;
    if (codegen_emit_args(c, n, func_reg, &argc)) {
        restore_regs(c, mark);
        return;
    }
    codegen_emit_call(c, n->as.call.callee, func_reg, argc, mode, n->line);
    /* mode 仅 0（RESULT_DISCARD）或 1（单值）——多返回值由赋值/返回路径单独发射，
       此处的 mode>1 多结果拷贝是死分支（2026-08-12 确认） */
    if (mode == 1 && dest != func_reg) {
        emit(c, OP_MOV, dest, func_reg, 0, n->line);
    }
    restore_regs(c, mark);
}

static void codegen_string(Codegen *c, const AstNode *n, int dest) {
    const size_t np = n->as.str.n_parts;
    if (np == 1 && n->as.str.parts[0].is_literal) {
        const AstStrPart *p = &n->as.str.parts[0];
        ObjString *s = obj_string_new_permanent(c->gc, p->text, p->len);
        emit_const_op(c, dest, obj_string_val(s), n->line);
        return;
    }
    if (np == 0) {
        ObjString *e = obj_string_new_permanent(c->gc, "", 0);
        emit_const_op(c, dest, obj_string_val(e), n->line);
        return;
    }
    /* 多段插值：预留 np 个连续临时寄存器，各段编译到 [first, first+np)，
       一条 OP_CONCAT 一次拼接（各段字符串化，省 n-1 次中间串分配）。 */
    int mark = c->reg_top;
    int first = alloc_temp(c);
    for (size_t i = 1; i < np; i++) alloc_temp(c);
    if (np - 1 > 511) { /* OP_CONCAT 的 c 是 9 位（段数-1，0..511） */
        error(c, n, "too many interpolation segments");
        restore_regs(c, mark);
        return;
    }
    for (size_t i = 0; i < np; i++) {
        const AstStrPart *p = &n->as.str.parts[i];
        int t = first + (int)i;
        if (p->is_literal) {
            ObjString *s = obj_string_new_permanent(c->gc, p->text, p->len);
            int k = add_const(c, obj_string_val(s));
            emit(c, OP_KSTR, t, k, 0, n->line); /* 字符串段 → str 池 + OP_KSTR */
        } else {
            codegen_expr(c, p->expr, t);
        }
    }
    emit(c, OP_CONCAT, dest, first, (int)np - 1, n->line);
    restore_regs(c, mark);
}

static void codegen_ident(Codegen *c, const AstNode *n, int dest) {
    const char *name = n->as.ident.name;
    int slot = resolve_local(c, name);
    if (slot >= 0) {
        emit(c, OP_MOV, dest, slot, 0, n->line);
        if (name_is_computed(c, name)) emit_computed_call(c, n, dest);
        return;
    }
    int cap = resolve_capval(c, name);
    if (cap >= 0) {
        emit(c, OP_GET_CAPVAL, dest, cap, 1, n->line);
        if (name_is_computed(c, name)) emit_computed_call(c, n, dest);
        return;
    }
    /* 嵌套类：方法体/嵌套代码按名字引用外层函数声明的嵌套类 → 发 OP_CONST 类常量
       （嵌套类无对应全局，LOAD_GLOBAL 会 undefined）。顶层类不在此路径（走 LOAD_GLOBAL）。 */
    ClassEntry *ce = find_class_entry(c, name);
    if (ce && ce->nested) {
        emit(c, OP_CONST, dest, add_const(c, obj_class_val(ce->klass)), 0, n->line);
        return;
    }
    /* 方案 A：import 来的名字 → 全限定全局引用（sqrt → System.Math.sqrt、Math → System.Math）；
       否则走 global_name（模块内部名加前缀 / 裸全局）。均为全局标识符池。 */
    const char *qualified = import_map_get(c, name);
    int k = qualified ? global_ident(c, qualified) : global_name(c, name);
    emit(c, OP_LOAD_GLOBAL, dest, k, 0, n->line);
    if (name_is_computed(c, name)) emit_computed_call(c, n, dest);
}

static void codegen_array(Codegen *c, const AstNode *n, int dest) {
    int mark = c->reg_top;
    int first = alloc_temp(c);
    for (size_t i = 1; i < n->as.array.n; i++) alloc_temp(c); /* 预留全部元素槽，防嵌套字面量碰撞 */
    for (size_t i = 0; i < n->as.array.n; i++) {
        codegen_expr(c, n->as.array.elems[i], first + (int)i);
    }
    emit(c, OP_ARRAY, dest, (uint16_t)n->as.array.n, first, n->line);
    restore_regs(c, mark);
}

static void codegen_map(Codegen *c, const AstNode *n, int dest) {
    int mark = c->reg_top;
    int first = alloc_temp(c);
    for (size_t i = 1; i < 2 * n->as.map.n; i++) alloc_temp(c); /* 键值槽 */
    for (size_t i = 0; i < n->as.map.n; i++) {
        const AstNode *key = n->as.map.entries[i].key;
        if (key->kind == AST_IDENT) {
            /* 裸标识符键 → 字符串键（{k: 1} 的 k 是字面量键，不是变量求值） */
            ObjString *ks = obj_string_new_permanent(c->gc, key->as.ident.name,
                                                     strlen(key->as.ident.name));
            const int k = add_const(c, obj_string_val(ks));
            emit(c, OP_KSTR, first + 2 * (int)i, k, 0, key->line);
        } else if (key->kind == AST_STRING) {
            codegen_expr(c, key, first + 2 * (int)i);
        } else {
            /* 2026-08-11：map 键仅限字符串（字面量或裸标识符） */
            error(c, key, "map keys must be strings (quoted or bare identifier)");
            restore_regs(c, mark);
            return;
        }
        codegen_expr(c, n->as.map.entries[i].value, first + 2 * (int)i + 1);
    }
    emit(c, OP_MAP, dest, (uint16_t)n->as.map.n, first, n->line);
    restore_regs(c, mark);
}

static void codegen_index_get(Codegen *c, const AstNode *n, int dest) {
    int mark = c->reg_top;
    /* 超级指令 B：全局数组元素读 `global[i]` → OP_GLOAD_INDEX（融合 LOAD_GLOBAL+INDEX，
       省一次 dispatch。全局名编译期下标，运行期条目缓存 + 数组快路径）。 */
    if (n->as.index.obj->kind == AST_IDENT) {
        const char* gname = n->as.index.obj->as.ident.name;
        if (resolve_local(c, gname) < 0 && resolve_capval(c, gname) < 0) {
            ClassEntry *ce = find_class_entry(c, gname);
            if (!(ce && ce->nested)) {
                int tidx;
                /* 下标是纯局部：直接用其寄存器（免冗余 MOV；regalloc 无 coalescing） */
                if (expr_plain_reg(c, n->as.index.index, &tidx)) {
                    /* 已在下标寄存器 */
                } else {
                    tidx = alloc_temp(c);
                    codegen_expr(c, n->as.index.index, tidx);
                }
                /* 与 codegen_ident 一致：import 来的名先查 import_map 压平（factors →
                   globmod.factors），否则裸名/模块前缀。均为全局标识符池。 */
                const char *qualified = import_map_get(c, gname);
                int k = qualified ? global_ident(c, qualified) : global_name(c, gname);
                if (k <= 255) {
                    emit(c, OP_GLOAD_INDEX, dest, k, tidx, n->line);
                } else {
                    /* GLOAD_INDEX 的 b 是 8 位（全局下标 ≤255）；超限回退 LOAD_GLOBAL+INDEX。 */
                    int tglob = alloc_temp(c);
                    emit(c, OP_LOAD_GLOBAL, tglob, k, 0, n->line);
                    emit(c, OP_INDEX, dest, tglob, tidx, n->line);
                }
                restore_regs(c, mark);
                return;
            }
        }
    }
    /* 立即数下标：下标是数字常量且常量下标 ≤511 → GETI（免加载下标 + 免 val_is_num 检查） */
    {
        Value cv;
        int k;
        if (expr_const_value(c, n->as.index.index, &cv) && val_is_num(cv) &&
            (k = add_const(c, cv)) >= 0 && k <= 511) {
            int obj_reg;
            if (expr_plain_reg(c, n->as.index.obj, &obj_reg)) {
                emit(c, OP_GETI, dest, obj_reg, k, n->line);
            } else {
                int tobj = alloc_temp(c);
                codegen_expr(c, n->as.index.obj, tobj);
                emit(c, OP_GETI, dest, tobj, k, n->line);
            }
            restore_regs(c, mark);
            return;
        }
    }
    int obj_reg;
    if (expr_plain_reg(c, n->as.index.obj, &obj_reg)) {
        int tidx;
        /* 下标是纯局部：直接用其寄存器（免冗余 MOV） */
        if (expr_plain_reg(c, n->as.index.index, &tidx)) {
            /* 已在下标寄存器 */
        } else {
            tidx = alloc_temp(c);
            codegen_expr(c, n->as.index.index, tidx);
        }
        emit(c, OP_INDEX, dest, obj_reg, tidx, n->line);
        restore_regs(c, mark);
    } else {
        int tobj = alloc_temp(c);
        codegen_expr(c, n->as.index.obj, tobj);
        int tidx = alloc_temp(c);
        codegen_expr(c, n->as.index.index, tidx);
        emit(c, OP_INDEX, dest, tobj, tidx, n->line);
        restore_regs(c, mark);
    }
}

static void codegen_member_get(Codegen *c, const AstNode *n, int dest) {
    /* super.method：父类实例方法绑定到 self */
    if (n->as.member.obj->kind == AST_IDENT &&
        strcmp(n->as.member.obj->as.ident.name, "super") == 0) {
        if (!c->cur_class || !c->cur_class->superclass) {
            error(c, n, "super used outside a method with a superclass");
            return;
        }
        ObjClass *parent = c->cur_class->superclass;
        int mark = c->reg_top;
        int pclass = alloc_temp(c);
        emit(c, OP_CONST, pclass, add_const(c, obj_class_val(parent)), 0, n->line);
        int k = intern_name(c, n->as.member.name);
        emit(c, OP_GET_METHOD, dest, k, pclass, n->line); /* 普通方法闭包（调用场景由 codegen_call_callee 注入 self） */
        restore_regs(c, mark);
        return;
    }
    /* 方案 A：整体 import 的点分模块别名（Math）压平 → LOAD_GLOBAL "System.Math.member"，
       免命名空间遍历 + 免裸全局。须非局部/捕获（局部 Math 是实例成员，不走全限定）。 */
    if (n->as.member.obj->kind == AST_IDENT) {
        const char *oname = n->as.member.obj->as.ident.name;
        const char *qualified = import_map_get(c, oname);
        if (qualified && strchr(qualified, '.') && resolve_local(c, oname) < 0 &&
            resolve_capval(c, oname) < 0) {
            char buf[512];
            snprintf(buf, sizeof(buf), "%s.%s", qualified, n->as.member.name);
            int k = global_ident(c, buf);
            emit(c, OP_LOAD_GLOBAL, dest, k, 0, n->line);
            return;
        }
    }
    int mark = c->reg_top;
    int obj_reg;
    if (expr_plain_reg(c, n->as.member.obj, &obj_reg)) {
        int k = intern_name(c, n->as.member.name);
        emit(c, OP_GET_MEMBER, dest, k, obj_reg, n->line);
    } else {
        int tobj = alloc_temp(c);
        codegen_expr(c, n->as.member.obj, tobj);
        int k = intern_name(c, n->as.member.name);
        emit(c, OP_GET_MEMBER, dest, k, tobj, n->line);
        restore_regs(c, mark);
    }
}

/* ---------- 错误处理：try / catch / 错误值构造 ---------- */

/* 编译块，并把值写入 dest（catch 块 / match 臂 / get{} 块表达式等"值块"）。
   值块值 = 末句表达式语句的值（`;` 结尾的调用语句 no_value → 视为副作用，值 null）；
   末句控制流表达式（分支各自末句表达式）也作值。 */
static void codegen_block_value(Codegen *c, const AstNode *block, int dest) {
    emit(c, OP_NULL, dest, 0, 0, block->line);
    begin_scope(c);
    const size_t n = block->as.block.n;
    for (size_t i = 0; i < n; i++) {
        const AstNode *s = block->as.block.stmts[i];
        if (i == n - 1 && s->kind == AST_EXPR_STMT && !s->as.expr_stmt.no_value) {
            codegen_expr(c, s->as.expr_stmt.expr, dest);
        } else if (i == n - 1 &&
                   (s->kind == AST_IF || s->kind == AST_WHILE || s->kind == AST_FOR_IN)) {
            codegen_expr(c, s, dest);
        } else {
            codegen_stmt(c, s);
        }
        if (c->has_error) break;
    }
    end_scope(c);
}

/* 编译块：value_dest >= 0 时作为"值块"（最后表达式写入 value_dest），
   否则作为普通语句块。if/while/for 既可用作语句也可用作表达式。 */
static void codegen_block_into(Codegen *c, const AstNode *block, int value_dest) {
    if (value_dest >= 0) codegen_block_value(c, block, value_dest);
    else codegen_block_stmt(c, block);
}

/* `Set.Case(args...)`：已知错误集的 case 构造 → OP_NEW_ERROR_NAMED。
   返回 true 表示已处理。 */
static bool codegen_error_case_construct(Codegen *c, const AstNode *call, int dest, uint16_t mode) {
    const AstNode *callee = call->as.call.callee;
    if (callee->kind != AST_MEMBER || callee->as.member.obj->kind != AST_IDENT) return false;
    const char *set_name = callee->as.member.obj->as.ident.name;
    const ErrorSetInfo *set = find_error_set(c, set_name);
    if (!set) return false;
    const AstErrorCase *ec = find_error_case(set, callee->as.member.name);
    if (!ec) return false;

    if (mode != 1 && mode != RESULT_DISCARD) {
        error(c, call, "error value construction yields a single value");
        return true;
    }
    const size_t n_args = call->as.call.n_args;
    int mark = c->reg_top;
    int result = alloc_temp(c);
    for (size_t i = 0; i < n_args; i++) alloc_temp(c); /* 载荷槽（result+1..） */
    for (size_t i = 0; i < n_args; i++) {
        const AstArg *a = &call->as.call.args[i];
        if (a->named) {
            error(c, call, "named arguments are not supported in v1");
            restore_regs(c, mark);
            return true;
        }
        codegen_expr(c, a->expr, result + 1 + (int)i);
    }
    emit(c, OP_NEW_ERROR_NAMED, result, (int)n_args,
         n_args > 0 ? result + 1 : 0, call->line); /* c=首载荷寄存器；无载荷时 0（避免发射
        帧外幻影寄存器 result+1 → regalloc color[] 越界读，2026-08-16 ASAN 修复） */
    int set_k = intern_const(c, set_name);
    int case_k = intern_const(c, ec->name);
    emit(c, OP_ERROR_INFO, set_k, case_k, 0, call->line);
    for (size_t i = 0; i < n_args; i++) {
        int name_k = (i < ec->n_payload && ec->payload[i]) ? intern_const(c, ec->payload[i])
                                                        : add_const(c, val_null());
        emit(c, OP_ERROR_INFO, name_k, 0, 0, call->line);
    }
    if (mode == 1 && dest != result) emit(c, OP_MOV, dest, result, 0, call->line);
    restore_regs(c, mark);
    return true;
}

/* 判断 AST_CALL 是否是已知错误集的 case 构造（`Set.Case(...)`），不发射代码。 */
static bool is_error_case_construct(Codegen *c, const AstNode *call) {
    if (call->kind != AST_CALL) return false;
    const AstNode *callee = call->as.call.callee;
    if (callee->kind != AST_MEMBER || callee->as.member.obj->kind != AST_IDENT) return false;
    const ErrorSetInfo *set = find_error_set(c, callee->as.member.obj->as.ident.name);
    if (!set) return false;
    return find_error_case(set, callee->as.member.name) != NULL;
}

/* `Color.Red(args...)`：已知枚举的带 payload case 构造 → OP_NEW_ENUM_NAMED。
   返回 true 表示已处理。 */
static bool codegen_enum_case_construct(Codegen *c, const AstNode *call, int dest, uint16_t mode) {
    const AstNode *callee = call->as.call.callee;
    if (callee->kind != AST_MEMBER || callee->as.member.obj->kind != AST_IDENT) return false;
    const char *set_name = callee->as.member.obj->as.ident.name;
    const EnumSetInfo *set = find_enum(c, set_name);
    if (!set) return false;
    const AstEnumMember *m = find_enum_case(set, callee->as.member.name);
    if (!m || m->n_payload == 0) return false;

    if (mode != 1 && mode != RESULT_DISCARD) {
        error(c, call, "enum value construction yields a single value");
        return true;
    }
    const size_t n_args = call->as.call.n_args;
    if (n_args != m->n_payload) {
        error(c, call, "enum case '%s.%s' expects %zu payload value(s), got %zu",
              set_name, m->name, m->n_payload, n_args);
        return true;
    }
    int mark = c->reg_top;
    int result = alloc_temp(c);
    for (size_t i = 0; i < n_args; i++) alloc_temp(c); /* 载荷槽（result+1..） */
    for (size_t i = 0; i < n_args; i++) {
        const AstArg *a = &call->as.call.args[i];
        if (a->named) {
            error(c, call, "named arguments are not supported for enum case construction");
            restore_regs(c, mark);
            return true;
        }
        codegen_expr(c, a->expr, result + 1 + (int)i);
    }
    emit(c, OP_NEW_ENUM_NAMED, result, (int)n_args,
         n_args > 0 ? result + 1 : 0, call->line); /* 防御：无载荷时 c=0（与 NEW_ERROR_NAMED 一致） */
    emit(c, OP_ENUM_INFO, intern_const(c, set_name), intern_const(c, m->name), 0, call->line);
    if (mode == 1 && dest != result) emit(c, OP_MOV, dest, result, 0, call->line);
    restore_regs(c, mark);
    return true;
}

static void codegen_error_val(Codegen *c, const AstNode *n, int dest) {
    int mark = c->reg_top;
    int t = alloc_temp(c);
    codegen_expr(c, n->as.error_val.msg, t);
    emit(c, OP_NEW_ERROR, dest, t, 0, n->line);
    restore_regs(c, mark);
}

/* 记录 do-catch 错误分支的 JMP 位置（catch 入口确定后统一 patch）。 */
static void catch_site_add_jump(CatchSite *site, size_t pos) {
    if (site->n_jumps == site->cap_jumps) {
        size_t ncap = site->cap_jumps ? site->cap_jumps * 2 : 4;
        site->jumps = realloc(site->jumps, ncap * sizeof(size_t));
        assert(site->jumps != NULL);
        site->cap_jumps = ncap;
    }
    site->jumps[site->n_jumps++] = pos;
}

static void codegen_try(Codegen *c, const AstNode *n, int dest) {
    const int kind = n->as.try_.kind;
    int mark = c->reg_top;
    c->protect_depth++; /* try 的 expr 内调用受保护（错误由 try 处理） */
    codegen_expr(c, n->as.try_.expr, dest);
    c->protect_depth--;
    int check = alloc_temp(c);
    emit(c, OP_IS_ERROR, check, dest, 0, n->line);
    size_t skip = proto_code_len(c->proto);
    emit_jump(c, OP_JMP_IF_FALSE, check, 0, n->line);
    if (c->catch_chain && kind != TOK_TRY_QUESTION) {
        /* do-catch 内：try/try! 的错误重定向到 catch 入口（错误值搬入 err_reg） */
        emit(c, OP_MOV, c->catch_chain->err_reg, dest, 0, n->line);
        size_t j = proto_code_len(c->proto);
        emit(c, OP_JMP, 0, 0, 0, n->line);
        catch_site_add_jump(c->catch_chain, j);
    } else if (kind == TOK_TRY) {
        if (c->enclosing == NULL) emit(c, OP_THROW, dest, 0, 0, n->line);
        else emit(c, OP_RETURN, dest, 0, 1, n->line);
    } else if (kind == TOK_TRY_QUESTION) {
        emit(c, OP_NULL, dest, 0, 0, n->line);
    } else { /* TOK_TRY_BANG */
        emit(c, OP_THROW, dest, 0, 0, n->line);
    }
    patch_jump(c, skip, proto_code_len(c->proto));
    restore_regs(c, mark);
}

/* try { stmts } / try! { stmts }：块内表达式语句的结果若为 error 则上抛/终止。
   只处理块内直接子语句中的 AST_EXPR_STMT（用户例：try { func1(); func2() }）；
   嵌套控制流内的调用不被该块捕获（v1 限制）。 */
static void codegen_try_stmt(Codegen *c, const AstNode *s, int kind) {
    if (s->kind == AST_EXPR_STMT && s->as.expr_stmt.expr) {
        int mark = c->reg_top;
        int t = alloc_temp(c);
        codegen_expr(c, s->as.expr_stmt.expr, t);
        int check = alloc_temp(c);
        emit(c, OP_IS_ERROR, check, t, 0, s->line);
        size_t skip = proto_code_len(c->proto);
        emit_jump(c, OP_JMP_IF_FALSE, check, 0, s->line);
        if (c->catch_chain) {
            emit(c, OP_MOV, c->catch_chain->err_reg, t, 0, s->line);
            size_t j = proto_code_len(c->proto);
            emit(c, OP_JMP, 0, 0, 0, s->line);
            catch_site_add_jump(c->catch_chain, j);
        } else if (kind == TOK_TRY) {
            if (c->enclosing == NULL) emit(c, OP_THROW, t, 0, 0, s->line);
            else emit(c, OP_RETURN, t, 0, 1, s->line);
        } else { /* TOK_TRY_BANG */
            emit(c, OP_THROW, t, 0, 0, s->line);
        }
        patch_jump(c, skip, proto_code_len(c->proto));
        restore_regs(c, mark);
    } else {
        codegen_stmt(c, s);
    }
}

static void codegen_try_block(Codegen *c, const AstNode *n) {
    const int kind = n->as.try_block.kind;
    int mark = c->reg_top;
    c->protect_depth++; /* 块内调用受保护（错误由 try 处理） */
    const size_t n_stmts = n->as.try_block.body->as.block.n;
    for (size_t i = 0; i < n_stmts; i++) {
        codegen_try_stmt(c, n->as.try_block.body->as.block.stmts[i], kind);
        if (c->has_error) break;
    }
    c->protect_depth--;
    restore_regs(c, mark);
}

/* do { stmts } catch <handler>：语句（不可作表达式）。块内 try/throw 的错误分支重定向到
   catch 入口（codegen_try/codegen_throw 的 catch_chain 分支），catch 模式匹配错误值。 */
static void codegen_do_catch(Codegen *c, const AstNode *n) {
    const int line = n->line;
    int mark = c->reg_top;
    int err = alloc_temp(c); /* 错误值约定寄存器 */
    CatchSite site = {0};
    site.err_reg = err;
    struct CatchSite *old = c->catch_chain;
    c->catch_chain = &site;
    c->protect_depth++; /* do 块内调用受保护（错误由 catch 处理） */
    codegen_block_stmt(c, n->as.do_catch.body);
    c->protect_depth--;
    /* 正常路径：跳到 catch 之后 */
    size_t done = proto_code_len(c->proto);
    emit(c, OP_JMP, 0, 0, 0, line);
    /* catch 入口：err 寄存器持有错误值 */
    const int catch_pos = (int)proto_code_len(c->proto);
    c->catch_chain = old;
    for (size_t i = 0; i < site.n_jumps; i++) patch_jump(c, site.jumps[i], catch_pos);

    const CatchHandler *h = &n->as.do_catch.handler;
    switch (h->kind) {
        case CATCH_MATCH:
            codegen_match_arms(c, h->arms, h->n_arms, err, err, line, true, false);
            break;
        case CATCH_BLOCK: {
            begin_scope(c);
            if (h->error_name) {
                int slot = add_local(c, h->error_name, true);
                emit(c, OP_MOV, slot, err, 0, line);
            }
            codegen_block_stmt(c, h->fallback);
            end_scope(c);
            break;
        }
        case CATCH_FALLBACK:
            codegen_expr(c, h->fallback, err);
            break;
    }
    patch_jump(c, done, proto_code_len(c->proto));
    free(site.jumps);
    restore_regs(c, mark);
}

/* a?.b / a?.b(args)：a 为 null → null（不访问 .b）；否则 a.b（或调用 a.b(args)）。
   编译：求 a 到 t；ISNE t,null → t!=null 跳过下一条；t==null 分支 OP_NULL。 */
static void codegen_optional(Codegen *c, const AstNode *n, int dest) {
    int mark = c->reg_top;
    int t = alloc_temp(c);
    codegen_expr(c, n->as.optional.obj, t);
    int nreg = alloc_temp(c);
    emit_const_op(c, nreg, val_null(), n->line);
    emit(c, OP_ISNE, t, nreg, 0, n->line); /* t != null → 跳过下一条 JMP */
    size_t jnull = proto_code_len(c->proto);
    emit(c, OP_JMP, 0, 0, 0, n->line);     /* t == null → null 分支 */
    /* t != null：成员访问或调用 */
    int rk = intern_name(c, n->as.optional.name);
    if (n->as.optional.is_call) {
        int fn = alloc_temp(c);
        emit(c, OP_GET_MEMBER, fn, rk, t, n->line); /* fn = t.name */
        int rself = alloc_temp(c);                  /* = fn+1：receiver 槽 */
        emit(c, OP_MOV, rself, t, 0, n->line);
        AstNode call_node = {0};
        call_node.kind = AST_CALL;
        call_node.line = n->line;
        call_node.as.call.args = n->as.optional.args;
        call_node.as.call.n_args = n->as.optional.n_args;
        uint16_t argc = 0;
        if (codegen_emit_args(c, &call_node, fn, &argc)) {
            restore_regs(c, mark);
            return;
        }
        const bool prot = c->protect_depth > 0;
        emit(c, OP_CALL, fn, argc, 1 | (prot ? RESULT_PROTECT : 0), n->line);
        if (dest != fn) emit(c, OP_MOV, dest, fn, 0, n->line);
    } else {
        emit(c, OP_GET_MEMBER, dest, rk, t, n->line);
    }
    size_t jdone = proto_code_len(c->proto);
    emit(c, OP_JMP, 0, 0, 0, n->line);
    /* null 分支 */
    patch_jump(c, jnull, (int)proto_code_len(c->proto));
    emit(c, OP_NULL, dest, 0, 0, n->line);
    patch_jump(c, jdone, (int)proto_code_len(c->proto));
    restore_regs(c, mark);
}

/* 字面量模式 → 通用常量池索引（OP_MATCH_LITERAL 读通用池，故强制 add_const_gen，
   即使值是 num/str 也进 constants——MATCH_LITERAL 的 vm 端只读 constants）。 */
static int add_literal_const(Codegen *c, const AstNode *lit) {
    switch (lit->kind) {
        case AST_NUMBER: {
            Value v = parse_number_text(c, lit->as.number.text, lit->as.number.len);
            return add_const_gen(c, v);
        }
        case AST_STRING: {
            if (lit->as.str.n_parts == 1 && lit->as.str.parts[0].is_literal) {
                const AstStrPart *p = &lit->as.str.parts[0];
                return add_const_gen(c, obj_string_val(obj_string_new_permanent(c->gc, p->text, p->len)));
            }
            error(c, lit, "interpolated string patterns are not supported");
            return 0;
        }
        case AST_TRUE: return add_const_gen(c, val_true());
        case AST_FALSE: return add_const_gen(c, val_false());
        case AST_NULL: return add_const_gen(c, val_null()); /* null 字面量：真实 null 常量（去重复用） */
        default:
            error(c, lit, "invalid literal pattern");
            return 0;
    }
}

/* 为 arm 的模式发射匹配检查：结果写入 match_reg（true=匹配）。match_error=true（catch）
   时枚举 case 匹配 ObjError（OP_MATCH_ERROR + OP_ERROR_INFO）；false（match）时匹配
   ObjEnumVal（OP_MATCH_ENUM + OP_ENUM_INFO）。返回 true 表示无条件匹配
   （ARM_BIND/ARM_WILDCARD，不发射检查）。 */
static bool codegen_pattern_match(Codegen *c, const AstMatchArm *arm, int value_reg,
                                  int match_reg, int line, bool match_error) {
    switch (arm->pattern) {
        case ARM_LITERAL: {
            int k = add_literal_const(c, arm->literal);
            emit(c, OP_MATCH_LITERAL, match_reg, value_reg, k, line);
            return false;
        }
        case ARM_ENUM: {
            const int set_k = arm->set_name ? intern_const(c, arm->set_name) : add_const(c, val_null());
            const int case_k = arm->case_name ? intern_const(c, arm->case_name) : add_const(c, val_null());
            if (match_error) {
                emit(c, OP_MATCH_ERROR, match_reg, value_reg, 0, line);
                emit(c, OP_ERROR_INFO, set_k, case_k, 0, line);
            } else {
                /* match：带 payload case 匹配 ObjEnumVal；无 payload case（整数成员）
                   匹配其静态整数值（OP_MATCH_LITERAL）。 */
                const EnumSetInfo *set = arm->set_name ? find_enum(c, arm->set_name) : NULL;
                const AstEnumMember *em = set ? find_enum_case(set, arm->case_name) : NULL;
                int64_t iv;
                if (em && em->n_payload == 0 && enum_member_int_value(c, set, em, &iv)) {
                    emit(c, OP_MATCH_LITERAL, match_reg, value_reg,
                         add_const_gen(c, val_float((double)iv)), line);
                } else {
                    emit(c, OP_MATCH_ENUM, match_reg, value_reg, set_k, line);
                    emit(c, OP_ENUM_INFO, add_const(c, val_null()), case_k, 0, line);
                }
            }
            return false;
        }
        case ARM_BIND:
        case ARM_WILDCARD:
            return true;
    }
    return true;
}

/* 通用匹配臂编译（catch 与 match 共用，解耦便于扩展复杂模式）：依次尝试各臂，
   命中求臂值到 dest 并跳到 done；全部未命中时 no_match_errors=true 发运行期错误
   "no matching pattern"，否则保持 value_reg 原值（catch 的"错误原样返回"语义）。
   match_error=true 时臂匹配/解构针对 ObjError（catch）。 */
static void codegen_match_arms(Codegen *c, const AstMatchArm *arms, size_t n_arms,
                               int value_reg, int dest, int line, bool match_error,
                               bool no_match_errors) {
    size_t *next_jumps = malloc((n_arms ? n_arms : 1) * sizeof(size_t));
    size_t *done_jumps = malloc((n_arms ? n_arms : 1) * sizeof(size_t));
    size_t *arm_starts = malloc((n_arms ? n_arms : 1) * sizeof(size_t));
    assert(next_jumps && done_jumps && arm_starts);
    int match = alloc_temp(c);

    for (size_t i = 0; i < n_arms; i++) {
        const AstMatchArm *arm = &arms[i];
        arm_starts[i] = proto_code_len(c->proto);
        if (codegen_pattern_match(c, arm, value_reg, match, line, match_error)) {
            next_jumps[i] = (size_t)-1; /* 无条件匹配 */
        } else {
            next_jumps[i] = proto_code_len(c->proto);
            emit_jump(c, OP_JMP_IF_FALSE, match, 0, line);
        }
        const bool binds = (arm->pattern == ARM_BIND && arm->bind) || arm->n_payload > 0;
        if (binds) begin_scope(c);
        if (arm->pattern == ARM_BIND && arm->bind) {
            int slot = add_local(c, arm->bind, true);
            emit(c, OP_MOV, slot, value_reg, 0, line);
        }
        for (size_t j = 0; j < arm->n_payload; j++) {
            int slot = add_local(c, arm->payload[j], true);
            if (match_error) emit(c, OP_GET_ERROR_PAYLOAD, slot, value_reg, (int)j, line);
            else emit(c, OP_GET_ENUM_PAYLOAD, slot, value_reg, (int)j, line);
        }
        const AstNode *av = arm->value;
        if (av && av->kind == AST_BLOCK) codegen_block_value(c, av, dest);
        else codegen_expr(c, av, dest);
        if (binds) end_scope(c);
        done_jumps[i] = proto_code_len(c->proto);
        emit_jump(c, OP_JMP, 0, 0, line);
    }

    /* 未命中 fallthrough 落点：no_match_errors 时发运行期错误，否则保持 value_reg 原值 */
    const size_t fall_pos = proto_code_len(c->proto);
    if (no_match_errors) {
        int msg = intern_const(c, "no matching pattern");
        emit(c, OP_ERROR, msg, 0, 0, line);
    }
    /* done 跳到所有错误之后（命中臂的 JMP 不得落在 OP_ERROR 上） */
    const size_t done_pos = proto_code_len(c->proto);
    for (size_t i = 0; i < n_arms; i++) {
        if (next_jumps[i] != (size_t)-1) {
            const size_t target = (i + 1 < n_arms) ? arm_starts[i + 1] : fall_pos;
            patch_jump(c, next_jumps[i], (int)target);
        }
    }
    for (size_t i = 0; i < n_arms; i++) patch_jump(c, done_jumps[i], (int)done_pos);
    free(next_jumps);
    free(done_jumps);
    free(arm_starts);
}

/* catch 模式匹配：针对 error 值（未匹配保持原错误返回，不报错）。 */
static void codegen_catch_match(Codegen *c, const AstNode *n, int dest, int err_reg, int line) {
    const CatchHandler *h = &n->as.catch_.handler;
    codegen_match_arms(c, h->arms, h->n_arms, err_reg, dest, line, true, false);
}

static void codegen_catch(Codegen *c, const AstNode *n, int dest) {
    const CatchHandler *h = &n->as.catch_.handler;
    int mark = c->reg_top;
    c->protect_depth++; /* catch 的 expr 内调用受保护（错误由 catch 处理） */
    codegen_expr(c, n->as.catch_.expr, dest);
    c->protect_depth--;
    int check = alloc_temp(c);
    emit(c, OP_IS_ERROR, check, dest, 0, n->line);
    size_t skip = proto_code_len(c->proto);
    emit_jump(c, OP_JMP_IF_FALSE, check, 0, n->line);

    switch (h->kind) {
        case CATCH_FALLBACK:
            codegen_expr(c, h->fallback, dest);
            break;
        case CATCH_BLOCK: {
            begin_scope(c);
            if (h->error_name) {
                int slot = add_local(c, h->error_name, true);
                emit(c, OP_MOV, slot, dest, 0, n->line);
            }
            codegen_block_value(c, h->fallback, dest);
            end_scope(c);
            break;
        }
        case CATCH_MATCH:
            codegen_catch_match(c, n, dest, dest, n->line);
            break;
    }
    patch_jump(c, skip, proto_code_len(c->proto));
    restore_regs(c, mark);
}

/* `match expr { 模式 => 值, ... }`：表达式或语句（由调用方 value_dest 决定）。
   match 不能用于 error 类型（error 只能用 catch），运行期守卫。 */
static void codegen_match(Codegen *c, const AstNode *n, int dest) {
    int mark = c->reg_top;
    int t = alloc_temp(c);
    codegen_expr(c, n->as.match_.expr, t);
    int check = alloc_temp(c);
    emit(c, OP_IS_ERROR, check, t, 0, n->line);
    size_t ok = proto_code_len(c->proto);
    emit_jump(c, OP_JMP_IF_FALSE, check, 0, n->line);
    int msg = intern_const(c, "match cannot match an error value; use catch");
    emit(c, OP_ERROR, msg, 0, 0, n->line);
    patch_jump(c, ok, proto_code_len(c->proto));
    codegen_match_arms(c, n->as.match_.arms, n->as.match_.n_arms, t, dest, n->line, false, true);
    restore_regs(c, mark);
}

/* `throw expr`：函数内校验是 error 并作为返回值返回；顶层终止程序。 */
static void codegen_throw(Codegen *c, const AstNode *n) {
    int mark = c->reg_top;
    int t = alloc_temp(c);
    codegen_expr(c, n->as.throw_.expr, t);
    if (c->enclosing == NULL) {
        emit(c, OP_THROW, t, 0, 0, n->line); /* 顶层 throw：终止程序并打印错误 */
    } else {
        int check = alloc_temp(c);
        emit(c, OP_IS_ERROR, check, t, 0, n->line);
        size_t ok = proto_code_len(c->proto);
        emit_jump(c, OP_JMP_IF_TRUE, check, 0, n->line);
        int msg = intern_const(c, "throw: value is not an error");
        emit(c, OP_ERROR, msg, 0, 0, n->line);
        patch_jump(c, ok, proto_code_len(c->proto));
        if (c->catch_chain) {
            /* do-catch 内：throw 的错误重定向到 catch 入口 */
            emit(c, OP_MOV, c->catch_chain->err_reg, t, 0, n->line);
            size_t j = proto_code_len(c->proto);
            emit(c, OP_JMP, 0, 0, 0, n->line);
            catch_site_add_jump(c->catch_chain, j);
        } else {
            emit(c, OP_RETURN, t, 0, 1, n->line); /* 作为返回值返回 error 给调用方 */
        }
    }
    restore_regs(c, mark);
}

/* `x is T`：内置类型名（OP_IS_TYPE + 字符串常量）或类名（OP_IS_TYPE + ObjClass 常量）
   或枚举 case（`x is Color.Red` → OP_MATCH_ENUM）。 */
static void codegen_is(Codegen *c, const AstNode *n, int dest) {
    const AstNode *right = n->as.binary.right;
    int mark = c->reg_top;
    int v = alloc_temp(c);
    codegen_expr(c, n->as.binary.left, v);
    if (right->kind == AST_MEMBER) {
        if (right->as.member.obj->kind != AST_IDENT) {
            error(c, right, "right operand of 'is' must be a type name or enum case");
            restore_regs(c, mark);
            return;
        }
        const char *set_name = right->as.member.obj->as.ident.name;
        const char *case_name = right->as.member.name;
        const EnumSetInfo *set = find_enum(c, set_name);
        const AstEnumMember *em = set ? find_enum_case(set, case_name) : NULL;
        int64_t iv;
        if (em && em->n_payload == 0 && enum_member_int_value(c, set, em, &iv)) {
            emit(c, OP_MATCH_LITERAL, dest, v, add_const_gen(c, val_float((double)iv)), n->line);
        } else {
            emit(c, OP_MATCH_ENUM, dest, v, intern_const(c, set_name), n->line);
            emit(c, OP_ENUM_INFO, add_const(c, val_null()), intern_const(c, case_name), 0, n->line);
        }
    } else if (right->kind == AST_IDENT) {
        const char *name = right->as.ident.name;
        ObjClass *klass = find_class(c, name);
        int k;
        if (klass) {
            k = add_const_gen(c, obj_class_val(klass));
        } else {
            ObjString *s = obj_string_new_permanent(c->gc, name, strlen(name));
            k = add_const_gen(c, obj_string_val(s));
        }
        emit(c, OP_IS_TYPE, dest, v, k, n->line);
    } else {
        error(c, right, "right operand of 'is' must be a type name or enum case");
    }
    restore_regs(c, mark);
}

static void codegen_expr(Codegen *c, const AstNode *n, int dest) {
    if (c->has_error) return;
    switch (n->kind) {
        case AST_NUMBER: {
            Value v = parse_number_text(c, n->as.number.text, n->as.number.len);
            int64_t iv;
            if (val_is_num(v) && float_as_int(val_as_float(v), &iv))
                emit_int_const(c, dest, iv, n->line);   /* 整数 → MOVI/立即数 */
            else emit_const_op(c, dest, v, n->line);   /* float → KNUM */
            break;
        }
        case AST_STRING: codegen_string(c, n, dest); break;
        case AST_NULL: emit(c, OP_NULL, dest, 0, 0, n->line); break;
        case AST_TRUE: emit(c, OP_TRUE, dest, 0, 0, n->line); break;
        case AST_FALSE: emit(c, OP_FALSE, dest, 0, 0, n->line); break;
        case AST_UNINITIALIZED: emit(c, OP_NULL, dest, 0, 0, n->line); break;
        case AST_IDENT: codegen_ident(c, n, dest); break;
        case AST_UNARY: codegen_unary(c, n, dest); break;
        case AST_BINARY: codegen_binary(c, n, dest); break;
        case AST_TERNARY: codegen_ternary(c, n, dest); break;
        case AST_CALL: codegen_call(c, n, dest, 1); break;
        case AST_INDEX: codegen_index_get(c, n, dest); break;
        case AST_MEMBER: codegen_member_get(c, n, dest); break;
        case AST_ARRAY: codegen_array(c, n, dest); break;
        case AST_MAP: codegen_map(c, n, dest); break;
        case AST_FUNC_DECL: { /* 匿名函数作值 */
            ObjClosure *f = codegen_func_value(c, n);
            if (c->has_error) break;
            if (body_has_yield(n->as.func.body)) {
                codegen_coroutine(c, n->as.func.body, f, dest, n->line);
            } else {
                int k = add_const(c, obj_closure_val(f));
                emit(c, OP_CONST, dest, k, 0, n->line);
            }
            break;
        }
        case AST_LAMBDA:
            codegen_lambda(c, n, dest);
            break;
        case AST_THIS_FUNC:
            emit(c, OP_MOV, dest, 0, 0, n->line); /* 当前帧 slot0 = 函数/闭包自身 */
            break;
        case AST_VARARGS:
            if (!c->is_vararg) {
                error(c, n, "varargs '...' used outside a vararg function");
                break;
            }
            emit(c, OP_MOV, dest, c->varargs_slot, 0, n->line); /* 入口已捕获的数组 */
            break;
        case AST_YIELD:
            codegen_yield(c, n, dest);
            break;
        case AST_TRY:
            codegen_try(c, n, dest);
            break;
        case AST_OPTIONAL:
            codegen_optional(c, n, dest);
            break;
        case AST_CATCH:
            codegen_catch(c, n, dest);
            break;
        case AST_MATCH:
            codegen_match(c, n, dest);
            break;
        case AST_ERROR_VAL:
            codegen_error_val(c, n, dest);
            break;
        case AST_BLOCK_EXPR:
            codegen_block_value(c, n, dest);
            break;
        case AST_IF:
            codegen_if(c, n, dest);
            break;
        case AST_WHILE:
            codegen_while(c, n, dest);
            break;
        case AST_FOR_IN:
            codegen_for(c, n, dest);
            break;
        default:
            error(c, n, "expression not supported in v1");
            break;
    }
}

/* ---------- 函数值（嵌套编译器） ---------- */

/* 默认参数 prologue：若参数为 null 则求默认表达式。参数是普通寄存器（开放捕获值模型）。
   has_self_param=true（类方法）：self 是 params[0]@slot1，声明参数 slot 1+i；
   false（普通/lambda）：slot1=null self 保留，声明参数 slot 2+i。 */
static void codegen_default_prologue(Codegen *c, const AstParam *params, size_t n_params,
                                     bool has_self_param, int line) {
    for (size_t i = 0; i < n_params; i++) {
        const AstParam *p = &params[i];
        if (!p->has_default) continue;
        const int slot = 2 + (int)i - (has_self_param ? 1 : 0);
        int mark = c->reg_top;
        int tcheck = alloc_temp(c);
        emit(c, OP_MOV, tcheck, slot, 0, line);
        int tnull = alloc_temp(c);
        emit(c, OP_NULL, tnull, 0, 0, line); /* 默认参数判 null：OP_NULL 免常量池 */
        emit(c, OP_EQ, tcheck, tcheck, tnull, line);
        size_t jskip = proto_code_len(c->proto);
        emit_jump(c, OP_JMP_IF_FALSE, tcheck, 0, line);
        codegen_expr(c, p->default_expr, slot);
        patch_jump(c, jskip, proto_code_len(c->proto));
        restore_regs(c, mark);
    }
}

/* 编译函数/闭包体。is_lambda 决定是否捕获外层。
   输出 capvals（捕获描述数组，调用方释放）。 */
static ObjClosure *codegen_func_like(Codegen *c, const AstParam *params, size_t n_params,
                                  bool has_varargs, const AstNode *body, ObjString *func_name,
                                  bool is_lambda, bool is_static, bool is_throws, int line) {
    Codegen child = {0};
    child.gc = c->gc;
    child.peep = c->peep;
    child.enclosing = c;
    child.module_prefix = c->module_prefix;
    child.module_internal = c->module_internal;
    child.proto = proto_new();
    child.func_name = func_name;
    child.scope_depth = 0;
    child.reg_top = 1; /* slot 0 保留（函数值） */
    child.max_regs = 2; /* slot1 = self 恒保留（普通/lambda 为 null） */
    child.is_lambda = is_lambda;
    child.is_vararg = has_varargs;   /* 函数体里的 ... 收集多余实参 */
    child.varargs_slot = -1;
    child.err = c->err;
    /* 类方法：继承 cur_class；self 名 = 首个参数（静态方法无 self；lambda 继承外层方法的 self） */
    child.cur_class = c->cur_class;
    child.self_name = is_lambda ? c->self_name
                                : (c->cur_class && !is_static && n_params > 0 ? params[0].name : NULL);
    /* self 类推断：使 self.method(x=1) 命名参数可静态解析 */
    if (child.cur_class && child.self_name) {
        add_var_class(&child, child.self_name, child.cur_class->name->chars);
    }

    /* 统一调用约定（2026-08-15）：slot1 恒为 self。实例方法 self 是 params[0]（slot1）；
       普通函数/lambda/静态方法 slot1 = null self（保留）。声明参数从 slot2 排，arity 恒含 self。 */
    const bool has_self_param = (child.cur_class && !is_lambda && !is_static && n_params > 0);

    /* 捕获预扫描：先于 add_local，使装箱判定生效 */
    collect_captured_body(body, params, n_params, 1, &child.captured_names, child.self_name);

    if (has_self_param) {
        add_local(&child, params[0].name, false);   /* self@slot1 */
        for (size_t i = 1; i < n_params; i++) add_local(&child, params[i].name, false);
    } else {
        child.reg_top = 2;   /* slot1 = null self（保留，不入 locals） */
        for (size_t i = 0; i < n_params; i++) add_local(&child, params[i].name, false);
    }
    const uint8_t arity = (uint8_t)(n_params + (has_self_param ? 0 : 1)); /* 恒含 self */
    /* vararg 函数：预留 slot arity+1 存放多余实参数组，并在入口第一条指令捕获。
       必须先于任何局部变量写入（否则局部会覆盖实参槽）；该槽永不复用。 */
    if (has_varargs) {
        child.varargs_slot = arity + 1;
        child.reg_top = child.varargs_slot + 1;
        emit(&child, OP_VARARGS, child.varargs_slot, 0, 0, line);
    }
    /* 被捕获的参数是普通寄存器（开放捕获值模型：运行时 OP_CLOSURE 才建捕获值指向该槽） */
    codegen_default_prologue(&child, params, n_params, has_self_param, line);

    if (body && body->kind == AST_BLOCK) {
        begin_scope(&child);
        for (size_t i = 0; i < body->as.block.n; i++) {
            codegen_stmt(&child, body->as.block.stmts[i]);
            if (child.has_error) break;
        }
        end_scope(&child);
    } else if (body) {
        int t = alloc_temp(&child);
        codegen_expr(&child, body, t);
        emit(&child, OP_RETURN, t, 0, 1, line);
    }
    emit(&child, OP_RET0, 0, 0, 0, line);

    if (child.has_error) {
        c->has_error = true;
        name_set_free(&child.captured_names);
        proto_free(child.proto);
        free(child.capvals);
        free(child.locals);
        free(child.error_sets);
        free(child.enums);
        free(child.classes);
        free(child.func_sigs);
        free(child.var_classes);
        return NULL;
    }
    Proto *p = child.proto;
    p->name = func_name;
    p->arity = arity;
    p->max_regs = (uint8_t)child.max_regs;
    p->is_throws = is_throws;
    p->is_method = has_self_param; /* 类方法：arity 含 self；普通函数 arity 含隐藏 self 槽 */
    finish_func(p, c->peep);
    set_func_module(c->gc, c->module_prefix, p);
    /* 捕获描述落进 proto：OP_CLOSURE 的 cell 数 = n_capvals */
    if (child.n_capvals > 0) {
        p->capvals = malloc((size_t)child.n_capvals * sizeof(CapvalDesc));
        assert(p->capvals != NULL);
        for (int i = 0; i < child.n_capvals; i++) {
            p->capvals[i].is_local = child.capvals[i].is_local;
            p->capvals[i].index = (uint16_t)child.capvals[i].index;
        }
        p->n_capvals = (size_t)child.n_capvals;
    }
    free(child.capvals);
    name_set_free(&child.captured_names);
    free(child.locals);
    free(child.error_sets);
    free(child.enums);
    free(child.classes);
    return obj_closure_new_permanent(c->gc, p);
}

static ObjClosure *codegen_func_value(Codegen *c, const AstNode *func) {
    ObjString *name = func->as.func.name
                          ? obj_string_new_permanent(c->gc, func->as.func.name, strlen(func->as.func.name))
                          : NULL;
    return codegen_func_like(c, func->as.func.params, func->as.func.n_params,
                             func->as.func.has_varargs, func->as.func.body, name, false,
                             func->as.func.is_static, func->as.func.is_throws, func->line);
}

/* ---------- 类：编译期建 ObjClass ---------- */

/* pass 1：创建 ObjClass（含字段名），注册到类表。nested=true 表示函数体内的嵌套类。 */
static void codegen_class_create(Codegen *c, const AstNode *n, bool nested) {
    ObjString *name = obj_string_new_permanent(c->gc, n->as.class_.name, strlen(n->as.class_.name));
    ObjClass *klass = obj_class_new_permanent(c->gc, name);
    if (c->module_prefix) {
        klass->module = obj_string_new_permanent(c->gc, c->module_prefix, strlen(c->module_prefix));
    }
    ObjString **fnames = NULL;
    size_t nf = 0, cap = 0;
    for (size_t i = 0; i < n->as.class_.n_members; i++) {
        const AstNode *m = n->as.class_.members[i];
        if (m->kind == AST_VAR_DECL && m->as.var_decl.n_names > 0) {
            const char *fn = m->as.var_decl.names[0];
            if (nf == cap) {
                size_t ncap = cap ? cap * 2 : 8;
                fnames = realloc(fnames, ncap * sizeof(ObjString *));
                assert(fnames != NULL);
                cap = ncap;
            }
            fnames[nf++] = obj_string_new_permanent(c->gc, fn, strlen(fn));
        }
    }
    klass->field_names = fnames;
    klass->n_field_names = nf;
    add_class(c, n->as.class_.name, klass, n, nested);
}

/* pass 2：解析继承链（父类须已在 pass 1 创建），并构建字段下标表（继承前缀布局）。 */
static void codegen_class_resolve_super(Codegen *c, const AstNode *n) {
    ObjClass *klass = find_class(c, n->as.class_.name);
    if (!klass) return;
    if (n->as.class_.parent) {
        ObjClass *parent = find_class(c, n->as.class_.parent);
        if (!parent) {
            error(c, n, "unknown superclass");
            return;
        }
        klass->superclass = parent;
    }
    obj_class_build_field_index(c->gc, klass);
}

/* 生成字段默认值方法：func __defaults(self) { self.f = expr ... }
   寄存器从高水位开始分配，避免覆盖构造器传入的实参槽（构造器 after 链转发用）。 */
#define DEFAULTS_REG_START 64

static ObjClosure *codegen_defaults_func(Codegen *c, const AstNode *class_node) {
    Codegen child = {0};
    child.gc = c->gc;
    child.peep = c->peep;
    child.enclosing = c;
    child.module_prefix = c->module_prefix;
    child.module_internal = c->module_internal;
    child.proto = proto_new();
    child.func_name = obj_string_new_permanent(c->gc, "<defaults>", 10);
    child.scope_depth = 0;
    child.reg_top = 1;
    child.max_regs = 1;
    child.is_lambda = false;
    child.err = c->err;
    child.cur_class = c->cur_class;
    child.self_name = "self";
    add_local(&child, "self", false);   /* slot 1 */
    /* 临时寄存器从高水位开始，避免覆盖构造器传入的实参槽（after 链转发用） */
    child.reg_top = DEFAULTS_REG_START;
    child.max_regs = DEFAULTS_REG_START;

    {
        NameSet scope = {0};
        name_set_add(&scope, "self");
        for (size_t i = 0; i < class_node->as.class_.n_members; i++) {
            const AstNode *m = class_node->as.class_.members[i];
            if (m->kind == AST_VAR_DECL && m->as.var_decl.n_values > 0) {
                scan_expr(m->as.var_decl.values[0], &scope, &child.captured_names, 1, "self");
            }
        }
        name_set_free(&scope);
    }

    const int self_slot = 1;

    /* 先调用父类的字段默认值方法（若有），保证继承字段的默认值生效 */
    if (child.cur_class && child.cur_class->superclass &&
        child.cur_class->superclass->defaults) {
        int mark = child.reg_top;
        int func = alloc_temp(&child);
        int arg = alloc_temp(&child);
        emit(&child, OP_MOV, arg, self_slot, 0, class_node->line);
        int k = add_const(&child, obj_closure_val(child.cur_class->superclass->defaults));
        emit(&child, OP_CONST, func, k, 0, class_node->line);
        emit(&child, OP_CALL, func, 1, RESULT_DISCARD, class_node->line);
        restore_regs(&child, mark);
    }

    for (size_t i = 0; i < class_node->as.class_.n_members; i++) {
        const AstNode *m = class_node->as.class_.members[i];
        if (m->kind != AST_VAR_DECL || m->as.var_decl.n_values == 0) continue;
        int mark = child.reg_top;
        int t = alloc_temp(&child);
        codegen_expr(&child, m->as.var_decl.values[0], t);
        int k = intern_name(&child, m->as.var_decl.names[0]);
        emit(&child, OP_SET_MEMBER, t, k, self_slot, m->line);
        restore_regs(&child, mark);
        if (child.has_error) break;
    }
    emit(&child, OP_RET0, 0, 0, RESULT_DISCARD, class_node->line);

    if (child.has_error) {
        c->has_error = true;
        name_set_free(&child.captured_names);
        proto_free(child.proto);
        free(child.locals);
        free(child.error_sets);
        free(child.enums);
        free(child.classes);
        free(child.func_sigs);
        free(child.var_classes);
        return NULL;
    }
    child.proto->name = child.func_name;
    child.proto->arity = 1;
    child.proto->is_method = true; /* __defaults(self)：方法（arity 含 self） */
    child.proto->max_regs = (uint8_t)child.max_regs;
    /* 注意：defaults 的寄存器从 DEFAULTS_REG_START=64 起分配，刻意避开构造器实参槽
       （after 链转发给 init 读）。图着色会压缩到低位覆盖实参 → 跳过。 */
    set_func_module(child.gc, child.module_prefix, child.proto);
    name_set_free(&child.captured_names);
    free(child.locals);
    free(child.error_sets);
    free(child.enums);
    free(child.classes);
    return obj_closure_new_permanent(child.gc, child.proto);
}

/* pass 3：编译类方法 + 字段默认值，填入 ObjClass。 */
static void codegen_class_build(Codegen *c, const AstNode *n) {
    ObjClass *klass = find_class(c, n->as.class_.name);
    if (!klass) return;
    ObjClass *saved = c->cur_class; /* 保存外层类，方法编译完恢复（嵌套类/字段默认值用） */
    c->cur_class = klass;
    /* has_getters/has_setters 沿继承链：父类有属性则子类也要查（obj_class_find_getter 沿链） */
    klass->has_getters = (klass->superclass && klass->superclass->has_getters);
    klass->has_setters = (klass->superclass && klass->superclass->has_setters);

    bool has_defaults = false;
    for (size_t i = 0; i < n->as.class_.n_members; i++) {
        const AstNode *m = n->as.class_.members[i];
        if (m->kind == AST_FUNC_DECL) {
            ObjClosure *f = codegen_func_value(c, m);
            if (c->has_error) break;
            ObjTable *table;
            if (m->as.func.is_property) {
                table = m->as.func.is_setter ? klass->setters : klass->getters;
                if (m->as.func.is_setter) klass->has_setters = true;
                else klass->has_getters = true;
            } else {
                table = m->as.func.is_static ? klass->static_methods : klass->methods;
            }
            objtable_set_kv(c->gc, table, fnv_hash(m->as.func.name, strlen(m->as.func.name)),
                            m->as.func.name, strlen(m->as.func.name), obj_closure_val(f));
        } else if (m->kind == AST_VAR_DECL && m->as.var_decl.n_values > 0) {
            has_defaults = true;
        }
    }
    /* 沿继承链：父类有字段默认值则本类也要生成 defaults（以调用父类 defaults） */
    if (!has_defaults) {
        for (ObjClass *p = klass->superclass; p != NULL; p = p->superclass) {
            if (p->defaults != NULL) { has_defaults = true; break; }
        }
    }
    if (!c->has_error && has_defaults) {
        ObjClosure *df = codegen_defaults_func(c, n);
        if (!c->has_error) klass->defaults = df;
    }
    c->cur_class = saved;
}

/* 嵌套类（函数体内 `class Local { }`）：编译期建 permanent ObjClass（方法/字段布局/继承
   编译期完成），类名注册为局部槽 + 发射 OP_CONST 把类值写入槽。后续 `Local(...)`（构造）、
   `Local.m()`（类命名空间成员）、`x is Local`（类型检查）经局部槽/类表解析。方法体经
   codegen_func_like 继承 cur_class=self 类型，方法不捕获外层局部（同嵌套 func 语义）。 */
static void codegen_nested_class(Codegen *c, const AstNode *n) {
    codegen_class_create(c, n, true);
    if (c->has_error) return;
    codegen_class_resolve_super(c, n);
    if (c->has_error) return;
    codegen_class_build(c, n);
    if (c->has_error) return;
    ObjClass *klass = find_class(c, n->as.class_.name);
    if (!klass) return;
    int slot = add_local(c, n->as.class_.name, true);
    int k = add_const(c, obj_class_val(klass));
    emit(c, OP_CONST, slot, k, 0, n->line);
}

/* `enum Name { A, B = 5, C }`：在入口初始化字节码里构建命名空间 map
   {A: 0, B: 5, C: 6, ...} 并存为全局。成员无显式值时自动递增（上一个成员 + 1，首成员从 0）。
   `Name.Member` 之后就是对该全局 map 的成员访问。 */
static void codegen_enum_build(Codegen *c, const AstNode *n) {
    int mark = c->reg_top;
    int rmap = alloc_temp(c);
    emit(c, OP_MAP, rmap, 0, 0, n->line); /* 空 map */
    int rprev = alloc_temp(c);
    int rone = alloc_temp(c);
    emit_int_const(c, rprev, -1, n->line); /* prev=-1：首成员+1=0 */
    emit_int_const(c, rone, 1, n->line);
    int rval = alloc_temp(c);
    for (size_t i = 0; i < n->as.enum_.n_members; i++) {
        const AstEnumMember *m = &n->as.enum_.members[i];
        if (m->n_payload > 0) {
            /* 带 payload 成员：map 里存 null 占位（`Color.Red(...)` 由 codegen 拦截
               为 OP_NEW_ENUM_NAMED；直接访问 `Color.Red` 得 null，与 error set 一致）。
               payload 成员不参与自动递增。 */
            emit(c, OP_NULL, rval, 0, 0, n->line);
            emit(c, OP_SET_MEMBER, rval, intern_name(c, m->name), rmap, n->line);
            if (c->has_error) return;
            continue;
        }
        if (m->value) {
            codegen_expr(c, m->value, rval);
        } else {
            emit(c, OP_ADD, rval, rprev, rone, n->line); /* prev + 1 */
        }
        emit(c, OP_MOV, rprev, rval, 0, n->line); /* prev = rval */
        emit(c, OP_SET_MEMBER, rval, intern_name(c, m->name), rmap, n->line);
        if (c->has_error) return;
    }
    emit(c, OP_STORE_GLOBAL, rmap, global_name(c, n->as.enum_.name), 0, n->line);
    restore_regs(c, mark);
}

/* lambda 表达式：编译其函数体，发射 OP_CLOSURE + 捕获描述。 */
static void codegen_lambda(Codegen *c, const AstNode *n, int dest) {
    ObjClosure *f = codegen_func_like(c, n->as.lambda.params, n->as.lambda.n_params,
                                   n->as.lambda.has_varargs, n->as.lambda.body, NULL, true, false,
                                   n->as.lambda.is_throws, n->line);
    if (c->has_error) return;
    if (body_has_yield(n->as.lambda.body)) {
        codegen_coroutine(c, n->as.lambda.body, f, dest, n->line);
        return;
    }
    int kf = add_const(c, obj_closure_val(f));
    emit(c, OP_CLOSURE, dest, kf, (int)f->proto->n_capvals, n->line);
}

static void codegen_yield(Codegen *c, const AstNode *n, int dest) {
    int mark = c->reg_top;
    int v = alloc_temp(c);
    if (n->as.yield_.expr) codegen_expr(c, n->as.yield_.expr, v);
    else emit(c, OP_NULL, v, 0, 0, n->line);
    emit(c, OP_YIELD, dest, v, 0, n->line);
    restore_regs(c, mark);
}

/* 函数体是否含 yield（不深入嵌套 func/lambda——它们各自是独立协程函数）。 */
static bool body_has_yield(const AstNode *body) {
    if (!body) return false;
    if (body->kind == AST_YIELD) return true;
    if (body->kind == AST_LAMBDA || body->kind == AST_FUNC_DECL || body->kind == AST_DEFER) {
        /* defer 块是闭包：其内 yield 属于该 defer 的协程函数，不算本函数 */
        return false;
    }
    if (body->kind == AST_BLOCK) {
        for (size_t i = 0; i < body->as.block.n; i++) {
            if (body_has_yield(body->as.block.stmts[i])) return true;
        }
        return false;
    }
    /* 通用表达式/语句递归（保守：深入所有子节点，但上面已挡掉 func/lambda） */
    switch (body->kind) {
        case AST_EXPR_STMT: return body_has_yield(body->as.expr_stmt.expr);
        case AST_THROW: return body_has_yield(body->as.throw_.expr);
        case AST_MATCH:
            if (body_has_yield(body->as.match_.expr)) return true;
            for (size_t i = 0; i < body->as.match_.n_arms; i++) {
                if (body->as.match_.arms[i].value && body_has_yield(body->as.match_.arms[i].value)) return true;
            }
            return false;
        case AST_CATCH:
            if (body_has_yield(body->as.catch_.expr)) return true;
            if (body->as.catch_.handler.kind == CATCH_FALLBACK ||
                body->as.catch_.handler.kind == CATCH_BLOCK) {
                return body_has_yield(body->as.catch_.handler.fallback);
            }
            for (size_t i = 0; i < body->as.catch_.handler.n_arms; i++) {
                if (body->as.catch_.handler.arms[i].value &&
                    body_has_yield(body->as.catch_.handler.arms[i].value)) return true;
            }
            return false;
        case AST_RETURN:
            for (size_t i = 0; i < body->as.return_.n; i++)
                if (body_has_yield(body->as.return_.values[i])) return true;
            return false;
        case AST_BLOCK_EXPR:
            for (size_t i = 0; i < body->as.block.n; i++)
                if (body_has_yield(body->as.block.stmts[i])) return true;
            return false;
        case AST_IF:
            if (body_has_yield(body->as.if_.cond)) return true;
            if (body->as.if_.then_ && body_has_yield(body->as.if_.then_)) return true;
            for (size_t i = 0; i < body->as.if_.n_elif; i++) {
                if (body_has_yield(body->as.if_.elif_conds[i])) return true;
                if (body->as.if_.elif_bodies[i] && body_has_yield(body->as.if_.elif_bodies[i])) return true;
            }
            return body->as.if_.else_ && body_has_yield(body->as.if_.else_);
        case AST_WHILE:
            return body_has_yield(body->as.while_.cond) || body_has_yield(body->as.while_.body);
        case AST_FOR_IN:
            if (body_has_yield(body->as.for_in.iterable)) return true;
            if (body->as.for_in.step && body_has_yield(body->as.for_in.step)) return true;
            return body_has_yield(body->as.for_in.body);
        case AST_VAR_DECL:
            for (size_t i = 0; i < body->as.var_decl.n_values; i++)
                if (body_has_yield(body->as.var_decl.values[i])) return true;
            return false;
        case AST_ASSIGN:
            for (size_t i = 0; i < body->as.assign.n; i++)
                if (body_has_yield(body->as.assign.targets[i])) return true;
            for (size_t i = 0; i < body->as.assign.n_values; i++)
                if (body_has_yield(body->as.assign.values[i])) return true;
            return false;
        case AST_UNARY: return body_has_yield(body->as.unary.operand);
        case AST_BINARY: return body_has_yield(body->as.binary.left) || body_has_yield(body->as.binary.right);
        case AST_TERNARY:
            return body_has_yield(body->as.ternary.cond) || body_has_yield(body->as.ternary.then_) ||
                   body_has_yield(body->as.ternary.else_);
        case AST_CALL:
            if (body_has_yield(body->as.call.callee)) return true;
            for (size_t i = 0; i < body->as.call.n_args; i++)
                if (body_has_yield(body->as.call.args[i].expr)) return true;
            return false;
        case AST_ARRAY:
            for (size_t i = 0; i < body->as.array.n; i++)
                if (body_has_yield(body->as.array.elems[i])) return true;
            return false;
        case AST_MAP:
            for (size_t i = 0; i < body->as.map.n; i++) {
                if (body_has_yield(body->as.map.entries[i].key)) return true;
                if (body_has_yield(body->as.map.entries[i].value)) return true;
            }
            return false;
        default:
            return false;
    }
}

/* 生成协程函数值：函数/闭包体含 yield → 置 proto->is_coroutine，
   工厂复用普通闭包（OP_CLOSURE，cell 数取 proto->n_capvals）。 */
static void codegen_coroutine(Codegen *c, const AstNode *body, ObjClosure *f, int dest, int line) {
    (void)body;
    f->proto->is_coroutine = true;
    int kf = add_const(c, obj_closure_val(f));
    emit(c, OP_CLOSURE, dest, kf, (int)f->proto->n_capvals, line);
}

/* ---------- defer ---------- */

/* 块内是否直接含 defer 语句（不含嵌套块）。 */
static bool block_has_direct_defer(const AstNode *block) {
    if (!block || block->kind != AST_BLOCK) return false;
    for (size_t i = 0; i < block->as.block.n; i++) {
        if (block->as.block.stmts[i]->kind == AST_DEFER) return true;
    }
    return false;
}

/* defer { block }：把块编译成闭包并压入当前帧 defer 栈。 */
static void codegen_defer(Codegen *c, const AstNode *n) {
    ObjClosure *f = codegen_func_like(c, NULL, 0, false, n->as.defer_.block,
                                   obj_string_new_permanent(c->gc, "<defer>", 7), true, false, false,
                                   n->line);
    if (c->has_error) return;
    int mark = c->reg_top;
    int t = alloc_temp(c);
    int kf = add_const(c, obj_closure_val(f));
    emit(c, OP_CLOSURE, t, kf, (int)f->proto->n_capvals, n->line);
    emit(c, OP_DEFER_PUSH, t, 0, 0, n->line);
    restore_regs(c, mark);
}

/* ---------- 语句 ---------- */

static void codegen_stmt(Codegen *c, const AstNode *n);

static void codegen_block_stmt(Codegen *c, const AstNode *block) {
    const bool has_defer = block_has_direct_defer(block);
    if (has_defer) {
        emit(c, OP_DEFER_MARK, 0, 0, 0, block->line);
        c->n_active_defers++;
    }
    begin_scope(c);
    for (size_t i = 0; i < block->as.block.n; i++) {
        codegen_stmt(c, block->as.block.stmts[i]);
        if (c->has_error) break;
    }
    end_scope(c);
    if (has_defer) {
        emit(c, OP_DEFER_RUN, 0, 0, 0, block->line);
        c->n_active_defers--;
    }
}

/* 把 src 寄存器（-1 = null）存入 target。 */
/* 向名字写入寄存器 src（-1=null）：处理装箱局部/capval/全局。 */
static void store_name(Codegen *c, const char *name, int src, int line) {
    int slot = resolve_local(c, name);
    if (slot >= 0) {
        if (src < 0) emit(c, OP_NULL, slot, 0, 0, line);
        else if (src != slot) emit(c, OP_MOV, slot, src, 0, line);
        return;
    }
    int cap = resolve_capval(c, name);
    if (cap >= 0) {
        if (src < 0) {
            int t = alloc_temp(c);
            emit(c, OP_NULL, t, 0, 0, line);
            emit(c, OP_SET_CAPVAL, t, cap, 1, line);
            c->reg_top--;
        } else {
            emit(c, OP_SET_CAPVAL, src, cap, 1, line);
        }
        return;
    }
    int t = src < 0 ? alloc_temp(c) : src;
    if (src < 0) emit(c, OP_NULL, t, 0, 0, line);
    int k = global_name(c, name);
    emit(c, OP_STORE_GLOBAL, t, k, 0, line);
    if (src < 0) c->reg_top--;
}

/* 读取名字到 dest：处理局部/capval/全局。 */
static void load_name(Codegen *c, const char *name, int dest, int line) {
    int slot = resolve_local(c, name);
    if (slot >= 0) {
        emit(c, OP_MOV, dest, slot, 0, line);
        return;
    }
    int cap = resolve_capval(c, name);
    if (cap >= 0) {
        emit(c, OP_GET_CAPVAL, dest, cap, 1, line);
        return;
    }
    int k = global_name(c, name);
    emit(c, OP_LOAD_GLOBAL, dest, k, 0, line);
}

static void store_target(Codegen *c, const AstNode *target, int src, int line) {
    switch (target->kind) {
        case AST_IDENT:
            store_name(c, target->as.ident.name, src, line);
            break;
        case AST_INDEX: {
            int mark = c->reg_top;
            /* 立即数下标写：下标是数字常量且常量下标 ≤255 → SETI（免加载下标 + 免类型检查） */
            Value cv;
            int k;
            if (expr_const_value(c, target->as.index.index, &cv) && val_is_num(cv) &&
                (k = add_const(c, cv)) >= 0 && k <= 511) {
                int obj_reg;
                if (expr_plain_reg(c, target->as.index.obj, &obj_reg)) {
                    emit(c, OP_SETI, src, obj_reg, k, line);
                } else {
                    int tobj = alloc_temp(c);
                    codegen_expr(c, target->as.index.obj, tobj);
                    emit(c, OP_SETI, src, tobj, k, line);
                }
                restore_regs(c, mark);
                break;
            }
            int obj_reg;
            int tidx;
            if (expr_plain_reg(c, target->as.index.obj, &obj_reg)) {
                tidx = alloc_temp(c);
                codegen_expr(c, target->as.index.index, tidx);
            } else {
                int tobj = alloc_temp(c);
                codegen_expr(c, target->as.index.obj, tobj);
                obj_reg = tobj;
                tidx = alloc_temp(c);
                codegen_expr(c, target->as.index.index, tidx);
            }
            emit(c, OP_INDEX_SET, src, obj_reg, tidx, line);
            restore_regs(c, mark);
            break;
        }
        case AST_MEMBER: {
            int mark = c->reg_top;
            int obj_reg;
            if (expr_plain_reg(c, target->as.member.obj, &obj_reg)) {
                int k = intern_name(c, target->as.member.name);
                emit(c, OP_SET_MEMBER, src, k, obj_reg, line);
            } else {
                int tobj = alloc_temp(c);
                codegen_expr(c, target->as.member.obj, tobj);
                int k = intern_name(c, target->as.member.name);
                emit(c, OP_SET_MEMBER, src, k, tobj, line);
                restore_regs(c, mark);
            }
            break;
        }
        default:
            error(c, target, "invalid assignment target");
            break;
    }
}

static void load_target(Codegen *c, const AstNode *target, int dest, int line) {
    switch (target->kind) {
        case AST_IDENT:
            load_name(c, target->as.ident.name, dest, line);
            break;
        case AST_INDEX: {
            int mark = c->reg_top;
            int obj_reg;
            int tidx;
            if (expr_plain_reg(c, target->as.index.obj, &obj_reg)) {
                tidx = alloc_temp(c);
                codegen_expr(c, target->as.index.index, tidx);
            } else {
                int tobj = alloc_temp(c);
                codegen_expr(c, target->as.index.obj, tobj);
                obj_reg = tobj;
                tidx = alloc_temp(c);
                codegen_expr(c, target->as.index.index, tidx);
            }
            emit(c, OP_INDEX, dest, obj_reg, tidx, line);
            restore_regs(c, mark);
            break;
        }
        case AST_MEMBER: {
            int mark = c->reg_top;
            int obj_reg;
            if (expr_plain_reg(c, target->as.member.obj, &obj_reg)) {
                int k = intern_name(c, target->as.member.name);
                emit(c, OP_GET_MEMBER, dest, k, obj_reg, line);
            } else {
                int tobj = alloc_temp(c);
                codegen_expr(c, target->as.member.obj, tobj);
                int k = intern_name(c, target->as.member.name);
                emit(c, OP_GET_MEMBER, dest, k, tobj, line);
                restore_regs(c, mark);
            }
            break;
        }
        default:
            error(c, target, "invalid assignment target");
            break;
    }
}

/* 声明局部并赋初值（src=-1=null）；被捕获的槽由 cap_floor 保留到帧返回。返回槽号。 */
static int define_local(Codegen *c, const char *name, bool is_const, int src, int line) {
    int slot = add_local(c, name, is_const);
    if (src < 0) emit(c, OP_NULL, slot, 0, 0, line);
    else if (slot != src) emit(c, OP_MOV, slot, src, 0, line);
    return slot;
}

/* 定义名字（局部/全局）为给定寄存器或 null（src=-1）。 */
static void define_name(Codegen *c, const char *name, bool is_const, int src, bool global,
                        int line) {
    if (global) {
        int t = src < 0 ? alloc_temp(c) : src;
        if (src < 0) emit(c, OP_NULL, t, 0, 0, line);
        int k = global_name(c, name);
        emit(c, OP_STORE_GLOBAL, t, k, 0, line);
        if (src < 0) c->reg_top--;
    } else {
        define_local(c, name, is_const, src, line);
    }
}

/* `let name => body`：只读计算属性 = 0 参闭包。存储后读取时自动调用（codegen_ident）。
   全局存 globals + computed_globals；局部存槽 + 标记 computed（槽先分配，避免 restore 覆盖）。 */
static void codegen_prop_decl(Codegen *c, const AstNode *n) {
    if (body_has_yield(n->as.prop_decl.body)) {
        error(c, n, "computed property cannot contain 'yield'");
        return;
    }
    const bool global = (c->enclosing == NULL && c->scope_depth == 0);
    if (global) {
        int mark = c->reg_top;
        int t = alloc_temp(c);
        ObjClosure *f = codegen_func_like(c, NULL, 0, false, n->as.prop_decl.body, NULL, true, false,
                                          false, n->line);
        if (c->has_error) { restore_regs(c, mark); return; }
        int kf = add_const(c, obj_closure_val(f));
        emit(c, OP_CLOSURE, t, kf, (int)f->proto->n_capvals, n->line);
        int k = global_name(c, n->as.prop_decl.name);
        emit(c, OP_STORE_GLOBAL, t, k, 0, n->line);
        name_set_add(&c->computed_globals, n->as.prop_decl.name);
        restore_regs(c, mark);
    } else {
        int slot = add_local(c, n->as.prop_decl.name, true); /* 槽先分配，reg_top 已越过它 */
        ObjClosure *f = codegen_func_like(c, NULL, 0, false, n->as.prop_decl.body, NULL, true, false,
                                          false, n->line);
        if (c->has_error) return;
        int kf = add_const(c, obj_closure_val(f));
        emit(c, OP_CLOSURE, slot, kf, (int)f->proto->n_capvals, n->line);
        mark_local_computed(c, n->as.prop_decl.name);
    }
}

static void codegen_var_decl(Codegen *c, const AstNode *n) {
    const size_t nn = n->as.var_decl.n_names;
    const size_t nv = n->as.var_decl.n_values;
    const bool global = (c->enclosing == NULL && c->scope_depth == 0);

    /* 多返回：let a, b = f() */
    if (nv == 1 && nn > 1 && n->as.var_decl.values[0]->kind == AST_CALL) {
        const AstNode *call = n->as.var_decl.values[0];
        int mark = c->reg_top;
        int func_reg = alloc_temp(c);
        codegen_call_callee(c, call->as.call.callee, func_reg, call->line);
        uint16_t argc = 0;
        if (codegen_emit_args(c, call, func_reg, &argc)) return;
        codegen_emit_call(c, call->as.call.callee, func_reg, argc, (uint16_t)nn, n->line);
        while (c->reg_top < func_reg + (int)nn) alloc_temp(c); /* 预留结果槽 */
        for (size_t i = 0; i < nn; i++) {
            define_name(c, n->as.var_decl.names[i], n->as.var_decl.is_const, func_reg + (int)i,
                        global, n->line);
        }
        if (global) restore_regs(c, mark); /* 仅全局时释放临时；局部槽保留 */
        return;
    }

    if (nv == nn) {
        /* 每个名字有自己的初值（顺序声明，允许前面的名字被后面引用） */
        for (size_t i = 0; i < nn; i++) {
            /* 推断 `var x = ClassName(...)`：记录 x → ClassName（命名参数解析实例方法用） */
            const char *cls = callee_class_name(c, n->as.var_decl.values[i]);
            if (cls && !find_var_class(c, n->as.var_decl.names[i])) {
                add_var_class(c, n->as.var_decl.names[i], cls);
            }
            if (global) {
                int mark = c->reg_top;
                int t = alloc_temp(c);
                codegen_expr(c, n->as.var_decl.values[i], t);
                int k = global_name(c, n->as.var_decl.names[i]);
                emit(c, OP_STORE_GLOBAL, t, k, 0, n->line);
                restore_regs(c, mark);
            } else {
                /* ⚠️ 必须先求初值再 add_local：旧实现先 add_local 再 codegen_expr，
                   于是 `var x = x + 1` 的初值解析到**刚声明的 x 自己**（甚至同名的
                   外层变量也被遮蔽）→ 读到未初始化槽/脏寄存器，静默错值。
                   ⚠️ 顺序同样是硬约束：**先 restore_regs 释放临时槽，再 add_local 占槽**。
                   反过来的话局部槽位于 reg_top 之上而 reg_top 已被回退，后续 alloc_temp
                   会复用该槽 → 局部的值被后续表达式的临时值覆盖（实测 `var w = 7` 之后
                   的调用参数变成 null）。 */
                int mark = c->reg_top;
                int t = alloc_temp(c);
                codegen_expr(c, n->as.var_decl.values[i], t);
                restore_regs(c, mark); /* 释放临时槽（先释放，避免 add_local 后槽被复用） */
                int slot = add_local(c, n->as.var_decl.names[i], n->as.var_decl.is_const);
                if (slot != t) emit(c, OP_MOV, slot, t, 0, n->line);
            }
        }
        return;
    }

    /* 名字多于值（如 let a, b = 1）：先求全部值，多余名字补 null */
    {
        int *vslots = malloc(nv ? nv * sizeof(int) : 1);
        assert(vslots != NULL);
        for (size_t i = 0; i < nv; i++) {
            vslots[i] = alloc_temp(c);
            codegen_expr(c, n->as.var_decl.values[i], vslots[i]);
        }
        for (size_t i = 0; i < nn; i++) {
            const char *name = n->as.var_decl.names[i];
            if (global) {
                int t = (i < nv) ? vslots[i] : alloc_temp(c);
                if (i >= nv) emit(c, OP_NULL, t, 0, 0, n->line);
                int k = global_name(c, name);
                emit(c, OP_STORE_GLOBAL, t, k, 0, n->line);
                if (i >= nv) c->reg_top--;
            } else {
                int slot = add_local(c, name, n->as.var_decl.is_const);
                if (i < nv) {
                    if (slot != vslots[i]) emit(c, OP_MOV, slot, vslots[i], 0, n->line);
                } else {
                    emit(c, OP_NULL, slot, 0, 0, n->line);
                }
            }
        }
        free(vslots);
        /* 临时槽在局部槽之下，无需释放（保留无害） */
    }
}

static void codegen_assign(Codegen *c, const AstNode *n) {
    const size_t nt = n->as.assign.n;
    const size_t nv = n->as.assign.n_values;
    const int op = n->as.assign.op;

    /* 单目标简单赋值：x = expr（经 store_name 以正确处理装箱局部/capval/全局） */
    if (op == TOK_ASSIGN && nt == 1 && n->as.assign.targets[0]->kind == AST_IDENT) {
        const AstNode *t = n->as.assign.targets[0];
        int slot = resolve_local(c, t->as.ident.name);
        if (slot >= 0) {
            /* 纯局部：直接求值到槽（免 temp + MOV；`x = x + 1` → ADDI x,x,1）。
               捕获局部也写槽（open-upval 指向槽，闭包读到新值）。
               ⚠️ 仅当 RHS 不求值用户代码（不会改写 x）时才可直接求值到槽；
               否则 `x = f()`（f 内读 x）会读到写入后的值，与左结合语义不符。 */
            if (!expr_may_call(n->as.assign.values[0])) {
                codegen_expr(c, n->as.assign.values[0], slot);
                return;
            }
            const int mark0 = c->reg_top;
            const int tmp0 = alloc_temp(c);
            codegen_expr(c, n->as.assign.values[0], tmp0);
            emit(c, OP_MOV, slot, tmp0, 0, n->line);
            restore_regs(c, mark0);
            return;
        }
        int mark = c->reg_top;
        int tmp = alloc_temp(c);
        codegen_expr(c, n->as.assign.values[0], tmp);
        store_name(c, t->as.ident.name, tmp, n->line);
        restore_regs(c, mark);
        return;
    }

    /* 多返回赋值：a, b = f() */
    if (op == TOK_ASSIGN && nv == 1 && nt > 1 && n->as.assign.values[0]->kind == AST_CALL) {
        const AstNode *call = n->as.assign.values[0];
        int mark = c->reg_top;
        int func_reg = alloc_temp(c);
        codegen_call_callee(c, call->as.call.callee, func_reg, call->line);
        uint16_t argc = 0;
        if (codegen_emit_args(c, call, func_reg, &argc)) return;
        codegen_emit_call(c, call->as.call.callee, func_reg, argc, (uint16_t)nt, n->line);
        for (size_t i = 0; i < nt; i++) {
            int src = func_reg + (int)i;
            store_target(c, n->as.assign.targets[i], src, n->line);
        }
        restore_regs(c, mark);
        return;
    }

    /* 通用：先求全部 RHS，再逐一存储 */
    int mark = c->reg_top;
    if (op == TOK_ASSIGN) {
        int *vslots = malloc(nv ? nv * sizeof(int) : 1);
        assert(vslots != NULL);
        for (size_t i = 0; i < nv; i++) {
            vslots[i] = alloc_temp(c);
            codegen_expr(c, n->as.assign.values[i], vslots[i]);
        }
        for (size_t i = 0; i < nt; i++) {
            int src = (i < nv) ? vslots[i] : -1;
            store_target(c, n->as.assign.targets[i], src, n->line);
        }
        free(vslots);
    } else {
        /* 复合赋值：load target → op rhs → store。目标为纯局部时直接操作其槽（免 load/store MOV）。 */
        if (nt != 1) {
            error(c, n, "compound assignment requires a single target");
        } else {
            OpCode opc = binop_to_opcode(op);
            if (opc == OP_COUNT) {
                error(c, n, "unsupported compound operator in v1");
            } else {
                int t = -1;
                bool direct_local = false;
                int obj_reg = -1, tidx = -1, mk = -1; /* 单次求值目标（index/member 读写复用） */
                bool single_eval = false;
                /* ⚠️ 复合赋值的语义 = `x = x <op> rhs`：目标的**读**必须发生在 RHS 求值之前。
                   若 RHS 可能调用用户代码（f() 内改写 x），旧实现"直接操作目标槽"会先算 RHS
                   再读 x → 得到 x_after op rhs（错误）。这里：RHS 可能有副作用时先 load 到
                   临时寄存器再算 RHS；纯字面量（无副作用）才保留直接操作槽的快路径。 */
                const bool rhs_may_call = expr_may_call(n->as.assign.values[0]);
                if (n->as.assign.targets[0]->kind == AST_IDENT) {
                    const int lslot = resolve_local(c, n->as.assign.targets[0]->as.ident.name);
                    if (lslot >= 0) {
                        if (!rhs_may_call) {
                            t = lslot;
                            direct_local = true; /* 直接操作槽（无副作用 RHS，安全且省 MOV） */
                        } else {
                            /* 先读目标到临时寄存器；下述 store_target 会在 RHS 之后写回 */
                            t = alloc_temp(c);
                            emit(c, OP_MOV, t, lslot, 0, n->line);
                            direct_local = false;
                            single_eval = false;
                        }
                    }
                }
                if (!direct_local && t < 0) {
                    /* 目标为 index/member（或未解析的 ident）：obj+下标只求值一次，
                       load/op/store 复用同一组寄存器——否则 load_target + store_target 各自
                       重新 codegen_expr 目标，副作用（调用）表达式
                       （arr[f()] += 1 / getObj().f -= 2）会被求值两次。 */
                    t = alloc_temp(c);
                    const AstNode *tg = n->as.assign.targets[0];
                    if (tg->kind == AST_INDEX) {
                        if (!expr_plain_reg(c, tg->as.index.obj, &obj_reg)) {
                            obj_reg = alloc_temp(c);
                            codegen_expr(c, tg->as.index.obj, obj_reg);
                        }
                        tidx = alloc_temp(c);
                        codegen_expr(c, tg->as.index.index, tidx);
                        emit(c, OP_INDEX, t, obj_reg, tidx, n->line);
                        single_eval = true;
                    } else if (tg->kind == AST_MEMBER) {
                        if (!expr_plain_reg(c, tg->as.member.obj, &obj_reg)) {
                            obj_reg = alloc_temp(c);
                            codegen_expr(c, tg->as.member.obj, obj_reg);
                        }
                        mk = intern_name(c, tg->as.member.name);
                        emit(c, OP_GET_MEMBER, t, mk, obj_reg, n->line);
                        single_eval = true;
                    } else {
                        load_target(c, tg, t, n->line);
                    }
                }
                Value cv;
                if ((opc == OP_ADD || opc == OP_SUB) &&
                    expr_const_value(c, n->as.assign.values[0], &cv) && val_is_num(cv) &&
                    val_as_float(cv) == 1.0) {
                    emit(c, (opc == OP_ADD) ? OP_INC : OP_DEC, t, 0, 0, n->line);
                } else {
                    /* 小整数常量（非 ±1）→ ADDI/SUBI（免加载常量进寄存器） */
                    Value cval;
                    int64_t iv;
                    if ((opc == OP_ADD || opc == OP_SUB) &&
                        expr_const_value(c, n->as.assign.values[0], &cval) && val_is_num(cval) &&
                        float_as_int(val_as_float(cval), &iv) && iv >= -128 && iv <= 127) {
                        emit(c, opc == OP_ADD ? OP_ADDI : OP_SUBI, t, t, (int)(int8_t)iv, n->line);
                    } else {
                        int t2 = alloc_temp(c);
                        codegen_expr(c, n->as.assign.values[0], t2);
                        emit(c, opc, t, t, t2, n->line);
                    }
                }
                if (!direct_local) {
                    if (single_eval) {
                        if (n->as.assign.targets[0]->kind == AST_INDEX) {
                            emit(c, OP_INDEX_SET, t, obj_reg, tidx, n->line);
                        } else { /* AST_MEMBER */
                            emit(c, OP_SET_MEMBER, t, mk, obj_reg, n->line);
                        }
                    } else {
                        store_target(c, n->as.assign.targets[0], t, n->line);
                    }
                }
            }
        }
    }
    restore_regs(c, mark);
}

static void codegen_expr_stmt(Codegen *c, const AstNode *n) {
    const AstNode *e = n->as.expr_stmt.expr;
    if (e->kind == AST_CALL) {
        codegen_call(c, e, 0, RESULT_DISCARD);
    } else {
        int mark = c->reg_top;
        int t = alloc_temp(c);
        codegen_expr(c, e, t);
        restore_regs(c, mark);
    }
}

static void codegen_break(Codegen *c, const AstNode *n) {
    if (!c->loops) {
        error(c, n, "break outside loop");
        return;
    }
    LoopInfo *l = c->loops;
    /* 弹出循环内活动的 defer 块（LIFO），再跳转 */
    for (int i = c->n_active_defers - l->defer_base; i > 0; i--) {
        emit(c, OP_DEFER_RUN, 0, 0, 0, n->line);
    }
    if (l->n_breaks == l->cap_breaks) {
        size_t ncap = l->cap_breaks ? l->cap_breaks * 2 : 4;
        l->breaks = realloc(l->breaks, ncap * sizeof(int));
        assert(l->breaks != NULL);
        l->cap_breaks = ncap;
    }
    l->breaks[l->n_breaks++] = (int)proto_code_len(c->proto);
    emit_jump(c, OP_JMP, 0, 0, n->line);
}

static void codegen_continue(Codegen *c, const AstNode *n) {
    if (!c->loops) {
        error(c, n, "continue outside loop");
        return;
    }
    LoopInfo *l = c->loops;
    for (int i = c->n_active_defers - l->defer_base; i > 0; i--) {
        emit(c, OP_DEFER_RUN, 0, 0, 0, n->line);
    }
    if (l->n_cont == l->cap_cont) {
        size_t ncap = l->cap_cont ? l->cap_cont * 2 : 4;
        l->continues = realloc(l->continues, ncap * sizeof(int));
        assert(l->continues != NULL);
        l->cap_cont = ncap;
    }
    l->continues[l->n_cont++] = (int)proto_code_len(c->proto);
    emit_jump(c, OP_JMP, 0, 0, n->line);
}

/* 阶段 C：条件跳转。简单比较条件（`a < b` 等）→ ISLT+JMP（比较-跳过，无结果寄存器）；
   否则回退 寄存器 bool + JMP_IF_FALSE。返回"条件假时跳转"的指令
   位置（调用方 patch 到 else/循环退出）。 */
static size_t codegen_cond_branch(Codegen *c, const AstNode *cond, int line) {
    int mark = c->reg_top;
    if (cond->kind == AST_BINARY) {
        OpCode isop = OP_COUNT;
        switch (binop_to_opcode(cond->as.binary.op)) {
            case OP_LT: isop = OP_ISLT; break;
            case OP_LE: isop = OP_ISLE; break;
            case OP_EQ: isop = OP_ISEQ; break;
            case OP_NE: isop = OP_ISNE; break;
            default: break;
        }
        if (isop != OP_COUNT) {
            /* 类型特化比较：`x == 字面量` / `x != 字面量`（数字/字符串常量）→ ISEQN/ISEQS 等
               （免加载字面量进寄存器 + 单类型检查）。ISEQN 族是 AD（d=16 位常量下标），
               add_const 超池限返回 -1 回退。 */
            if (isop == OP_ISEQ || isop == OP_ISNE) {
                int t_reg = -1;
                for (int side = 0; side < 2; side++) {
                    Value cv;
                    const AstNode *var_side = side == 0 ? cond->as.binary.left : cond->as.binary.right;
                    const AstNode *const_side = side == 0 ? cond->as.binary.right : cond->as.binary.left;
                    if (!expr_const_value(c, const_side, &cv)) continue;
                    OpCode cop = OP_COUNT;
                    int k;
                    if (val_is_num(cv)) cop = isop == OP_ISEQ ? OP_ISEQN : OP_ISNEN;
                    else if (val_is_string(cv)) cop = isop == OP_ISEQ ? OP_ISEQS : OP_ISNES;
                    else continue;
                    if ((k = add_const(c, cv)) < 0) continue;
                    if (!expr_plain_reg(c, var_side, &t_reg)) {
                        t_reg = alloc_temp(c);
                        codegen_expr(c, var_side, t_reg);
                    }
                    proto_emit(c->proto, cop, t_reg, k, 0, line);   /* 真 → 跳过下一条 JMP */
                    size_t j = proto_code_len(c->proto);
                    emit_jump(c, OP_JMP, 0, 0, line);               /* 假 → 跳 target */
                    restore_regs(c, mark);
                    return j;
                }
            }
            int t0, t1;
            if (!expr_plain_reg(c, cond->as.binary.left, &t0)) {
                t0 = alloc_temp(c);
                codegen_expr(c, cond->as.binary.left, t0);
            }
            if (!expr_plain_reg(c, cond->as.binary.right, &t1)) {
                t1 = alloc_temp(c);
                codegen_expr(c, cond->as.binary.right, t1);
            }
            if (binop_swap_operands(cond->as.binary.op)) { /* GT/GE → 交换用 ISLT/ISLE */
                const int tmp = t0;
                t0 = t1;
                t1 = tmp;
            }
            proto_emit(c->proto, isop, t0, t1, 0, line);   /* 真 → 跳过下一条 JMP */
            size_t j = proto_code_len(c->proto);
            emit_jump(c, OP_JMP, 0, 0, line);              /* 假 → 跳 target */
            restore_regs(c, mark);
            return j;
        }
    }
    int t = alloc_temp(c);
    codegen_expr(c, cond, t);
    size_t j = proto_code_len(c->proto);
    emit_jump(c, OP_JMP_IF_FALSE, t, 0, line);
    restore_regs(c, mark);
    return j;
}

static void codegen_if(Codegen *c, const AstNode *n, int value_dest) {
    int mark = c->reg_top;
    if (value_dest >= 0) emit(c, OP_NULL, value_dest, 0, 0, n->line); /* 无分支命中时保持 null */
    size_t cur_jf = codegen_cond_branch(c, n->as.if_.cond, n->line);
    codegen_block_into(c, n->as.if_.then_, value_dest);

    int *end_jumps = NULL;
    size_t n_end = 0, cap_end = 0;
    /* then 分支末尾跳 end */
    {
        size_t j = proto_code_len(c->proto);
        emit_jump(c, OP_JMP, 0, 0, n->line);
        if (n_end == cap_end) {
            size_t ncap = cap_end ? cap_end * 2 : 4;
            end_jumps = realloc(end_jumps, ncap * sizeof(int));
            assert(end_jumps != NULL);
            cap_end = ncap;
        }
        end_jumps[n_end++] = (int)j;
    }

    for (size_t i = 0; i < n->as.if_.n_elif; i++) {
        patch_jump(c, cur_jf, proto_code_len(c->proto));
        cur_jf = codegen_cond_branch(c, n->as.if_.elif_conds[i], n->line);
        codegen_block_into(c, n->as.if_.elif_bodies[i], value_dest);
        size_t j = proto_code_len(c->proto);
        emit_jump(c, OP_JMP, 0, 0, n->line);
        if (n_end == cap_end) {
            size_t ncap = cap_end ? cap_end * 2 : 4;
            end_jumps = realloc(end_jumps, ncap * sizeof(int));
            assert(end_jumps != NULL);
            cap_end = ncap;
        }
        end_jumps[n_end++] = (int)j;
    }

    patch_jump(c, cur_jf, proto_code_len(c->proto));
    if (n->as.if_.else_) codegen_block_into(c, n->as.if_.else_, value_dest);

    size_t end = proto_code_len(c->proto);
    for (size_t i = 0; i < n_end; i++) {
        proto_patch_jump(c->proto, (size_t)end_jumps[i], (int)(end - (size_t)end_jumps[i] - 1));
    }
    free(end_jumps);
    restore_regs(c, mark);
}

static void codegen_while(Codegen *c, const AstNode *n, int value_dest) {
    int mark = c->reg_top;
    if (value_dest >= 0) emit(c, OP_NULL, value_dest, 0, 0, n->line); /* 未进入循环保持 null */
    size_t loop_start = proto_code_len(c->proto);
    LoopInfo loop = {0};
    loop.continue_target = (int)loop_start;
    loop.defer_base = c->n_active_defers;
    loop.enclosing = c->loops;
    c->loops = &loop;

    size_t jf = codegen_cond_branch(c, n->as.while_.cond, n->line);
    codegen_block_into(c, n->as.while_.body, value_dest);
    c->n_active_defers = loop.defer_base; /* break 提前弹出后归位 */
    emit_jump(c, OP_JMP, 0, (int)(loop_start) - (int)proto_code_len(c->proto) - 1, n->line);

    size_t end = proto_code_len(c->proto);
    patch_jump(c, jf, end);
    for (size_t i = 0; i < loop.n_breaks; i++) {
        proto_patch_jump(c->proto, (size_t)loop.breaks[i], (int)(end - (size_t)loop.breaks[i] - 1));
    }
    for (size_t i = 0; i < loop.n_cont; i++) {
        proto_patch_jump(c->proto, (size_t)loop.continues[i],
                         (int)((size_t)loop.continue_target - (size_t)loop.continues[i] - 1));
    }
    c->loops = loop.enclosing;
    free(loop.breaks);
    free(loop.continues);
    restore_regs(c, mark);
}

static void codegen_for_range(Codegen *c, const AstNode *n, const AstNode *range, int value_dest) {
    if (n->as.for_in.n_vars != 1) {
        error(c, n, "range for-in requires exactly one loop variable");
        return;
    }
    int mark = c->reg_top;
    if (value_dest >= 0) emit(c, OP_NULL, value_dest, 0, 0, n->line); /* 未进入循环保持 null */
    begin_scope(c);

    /* FORI/FORL 控制寄存器组 a..a+3：a=step, a+1=limit, a+2=counter, a+3=循环值（body 用）。
       单条指令控循环（LuaJIT FORL 式）：step==0 由 FORI 先行报错，无需循环内重复检查。 */
    const int a = alloc_temp(c);
    alloc_temp(c);   /* a+1 */
    alloc_temp(c);   /* a+2 */
    alloc_temp(c);   /* a+3 */
    codegen_expr(c, range->as.binary.left, a + 2);   /* start → counter */
    codegen_expr(c, range->as.binary.right, a + 1);  /* end → limit */
    if (n->as.for_in.step) codegen_expr(c, n->as.for_in.step, a);
    else emit_int_const(c, a, 1, n->line);
    restore_regs(c, a + 4);   /* 释放 step 计算的临时，保留控制组 */

    /* 循环变量绑定到 R[a+2]（counter）：每次迭代 FORL 递增后跳 body，body 里 i 为当前值；
       退出后 counter = 递增后的值（如 1..3 退出 i=4），与旧 range-for 闭包捕获语义一致。 */
    c->reg_top = a + 2;
    add_local(c, n->as.for_in.vars[0], true);

    /* FORI a → body（step≠0 检查 + 首次跳）；L_head: FORL a → body；
       L_done: JMP done（FORL 不满足时跳过 body 退出循环） */
    size_t j_fori = proto_code_len(c->proto);
    emit_jump(c, OP_FORI, a, 0, n->line);
    size_t j_forl = proto_code_len(c->proto);
    emit_jump(c, OP_FORL, a, 0, n->line);
    size_t j_done_exit = proto_code_len(c->proto);
    emit_jump(c, OP_JMP, 0, 0, n->line);

    /* body */
    LoopInfo loop = {0};
    loop.defer_base = c->n_active_defers;
    loop.enclosing = c->loops;
    c->loops = &loop;
    size_t body = proto_code_len(c->proto);
    patch_jump(c, j_fori, body);
    patch_jump(c, j_forl, body);
    codegen_block_into(c, n->as.for_in.body, value_dest);
    c->n_active_defers = loop.defer_base;

    /* continue 点：跳回 FORL（L_head）重新递增判断 */
    size_t cont = proto_code_len(c->proto);
    emit_jump(c, OP_JMP, 0, (int)(j_forl) - (int)proto_code_len(c->proto) - 1, n->line);

    size_t done = proto_code_len(c->proto);
    patch_jump(c, j_done_exit, done);
    loop.continue_target = (int)cont;
    for (size_t i = 0; i < loop.n_breaks; i++) {
        proto_patch_jump(c->proto, (size_t)loop.breaks[i], (int)(done - (size_t)loop.breaks[i] - 1));
    }
    for (size_t i = 0; i < loop.n_cont; i++) {
        proto_patch_jump(c->proto, (size_t)loop.continues[i],
                         (int)((size_t)cont - (size_t)loop.continues[i] - 1));
    }
    c->loops = loop.enclosing;
    free(loop.breaks);
    free(loop.continues);

    end_scope(c);
    restore_regs(c, mark);
}

static void codegen_for_array(Codegen *c, const AstNode *n, int value_dest) {
    if (n->as.for_in.n_vars != 1) {
        error(c, n, "array for-in requires exactly one loop variable");
        return;
    }
    int mark = c->reg_top;
    if (value_dest >= 0) emit(c, OP_NULL, value_dest, 0, 0, n->line); /* 未进入循环保持 null */
    begin_scope(c);
    int ivar = add_local(c, n->as.for_in.vars[0], true);

    int rarr = alloc_temp(c);
    codegen_expr(c, n->as.for_in.iterable, rarr);
    int ridx = alloc_temp(c);
    emit_int_const(c, ridx, 0, n->line);

    size_t loop_start = proto_code_len(c->proto);
    /* ISLT_LEN：真（ridx < len）→ 跳过下一条 JMP → 直接进 body；假（退出）→ JMP 跳 done。
       合并 LENGTH+ISGE+JMP_IF_TRUE 三条为两条，省 rlen/rcond 临时寄存器。 */
    emit(c, OP_ISLT_LEN, ridx, rarr, 0, n->line);
    size_t jdone = proto_code_len(c->proto);
    emit_jump(c, OP_JMP, 0, 0, n->line);
    emit(c, OP_INDEX, ivar, rarr, ridx, n->line); /* 直接写循环变量槽，省 telem */

    LoopInfo loop = {0};
    loop.defer_base = c->n_active_defers;
    loop.enclosing = c->loops;
    c->loops = &loop;
    codegen_block_into(c, n->as.for_in.body, value_dest);
    c->n_active_defers = loop.defer_base;

    size_t cont = proto_code_len(c->proto);
    int k1 = alloc_temp(c);
    emit_int_const(c, k1, 1, n->line);
    emit(c, OP_ADD, ridx, ridx, k1, n->line);
    emit_jump(c, OP_JMP, 0, (int)loop_start - (int)proto_code_len(c->proto) - 1, n->line);

    size_t done = proto_code_len(c->proto);
    patch_jump(c, jdone, done);
    loop.continue_target = (int)cont;
    for (size_t i = 0; i < loop.n_breaks; i++) {
        proto_patch_jump(c->proto, (size_t)loop.breaks[i], (int)(done - (size_t)loop.breaks[i] - 1));
    }
    for (size_t i = 0; i < loop.n_cont; i++) {
        proto_patch_jump(c->proto, (size_t)loop.continues[i],
                         (int)((size_t)cont - (size_t)loop.continues[i] - 1));
    }
    c->loops = loop.enclosing;
    free(loop.breaks);
    free(loop.continues);

    end_scope(c);
    restore_regs(c, mark);
}

/* `for k, v in map`：键值两变量遍历。
   lowering：keys = map.keys()（快照数组）→ 按下标迭代，v = map[k]。
   复用既有 OP_INDEX/GET_MEMBER/CALL，无需新指令。 */
static void codegen_for_map(Codegen *c, const AstNode *n, int value_dest) {
    if (n->as.for_in.n_vars != 2) {
        error(c, n, "map for-in requires exactly two loop variables");
        return;
    }
    int mark = c->reg_top;
    if (value_dest >= 0) emit(c, OP_NULL, value_dest, 0, 0, n->line); /* 未进入循环保持 null */
    begin_scope(c);
    int kvar = add_local(c, n->as.for_in.vars[0], true);
    int vvar = add_local(c, n->as.for_in.vars[1], true);

    int rmap = alloc_temp(c);
    codegen_expr(c, n->as.for_in.iterable, rmap);

    /* rkeys = map.keys()（内置方法调用，结果覆写调用槽；receiver 注入 slot1） */
    int rkeys = alloc_temp(c);
    int kk = intern_name(c, "keys");
    emit(c, OP_GET_MEMBER, rkeys, kk, rmap, n->line);
    int rself = alloc_temp(c); /* = rkeys+1：receiver 槽 */
    emit(c, OP_MOV, rself, rmap, 0, n->line);
    emit(c, OP_CALL, rkeys, 1, 1, n->line);

    int ridx = alloc_temp(c);
    emit_int_const(c, ridx, 0, n->line);

    size_t loop_start = proto_code_len(c->proto);
    /* ISLT_LEN：真（ridx < len）→ 跳过下一条 JMP → body；假（退出）→ JMP 跳 done。
       rkeys 是 keys() 快照数组，循环内长度不变，与旧 OP_LENGTH 快照语义等价。 */
    emit(c, OP_ISLT_LEN, ridx, rkeys, 0, n->line);
    size_t jdone = proto_code_len(c->proto);
    emit_jump(c, OP_JMP, 0, 0, n->line);
    emit(c, OP_INDEX, kvar, rkeys, ridx, n->line);   /* 直接写循环变量槽，省 tkey */
    emit(c, OP_INDEX, vvar, rmap, kvar, n->line);    /* kvar 直接作下标，省 tval */

    LoopInfo loop = {0};
    loop.defer_base = c->n_active_defers;
    loop.enclosing = c->loops;
    c->loops = &loop;
    codegen_block_into(c, n->as.for_in.body, value_dest);
    c->n_active_defers = loop.defer_base;

    size_t cont = proto_code_len(c->proto);
    int k1 = alloc_temp(c);
    emit_int_const(c, k1, 1, n->line);
    emit(c, OP_ADD, ridx, ridx, k1, n->line);
    emit_jump(c, OP_JMP, 0, (int)loop_start - (int)proto_code_len(c->proto) - 1, n->line);

    size_t done = proto_code_len(c->proto);
    patch_jump(c, jdone, done);
    loop.continue_target = (int)cont;
    for (size_t i = 0; i < loop.n_breaks; i++) {
        proto_patch_jump(c->proto, (size_t)loop.breaks[i], (int)(done - (size_t)loop.breaks[i] - 1));
    }
    for (size_t i = 0; i < loop.n_cont; i++) {
        proto_patch_jump(c->proto, (size_t)loop.continues[i],
                         (int)((size_t)cont - (size_t)loop.continues[i] - 1));
    }
    c->loops = loop.enclosing;
    free(loop.breaks);
    free(loop.continues);

    end_scope(c);
    restore_regs(c, mark);
}

static void codegen_for(Codegen *c, const AstNode *n, int value_dest) {
    const AstNode *iterable = n->as.for_in.iterable;
    if (iterable->kind == AST_BINARY && iterable->as.binary.op == TOK_DOT_DOT) {
        codegen_for_range(c, n, iterable, value_dest);
    } else if (n->as.for_in.n_vars == 2) {
        codegen_for_map(c, n, value_dest);
    } else {
        codegen_for_array(c, n, value_dest);
    }
}

static void codegen_return(Codegen *c, const AstNode *n) {
    const size_t nv = n->as.return_.n;
    int mark = c->reg_top;
    /* 规则：error 值必须用 throw 返回，禁止 return error(...) / return Set.Case(...) */
    if (nv == 1) {
        const AstNode *v = n->as.return_.values[0];
        if (v->kind == AST_ERROR_VAL || (v->kind == AST_CALL && is_error_case_construct(c, v))) {
            error(c, n, "use 'throw' instead of 'return' to propagate an error");
            restore_regs(c, mark);
            return;
        }
    }
    if (nv == 1 && n->as.return_.values[0]->kind == AST_CALL) {
        /* return f()：可变结果数（f 可能返回 error 或正常值，无法静态判断，保持允许）。
           尾调用（P1）：f 是静态已知用户函数且未受保护 → OP_TAILCALL 复用当前帧
           （消除压帧/弹帧；VM 对构造器/getter/非闭包帧自动回退普通调用）。
           安全网 OP_RETURN 兜底传播结果；known-throws 仍须编译期报错（同 codegen_emit_call）。 */
        const AstNode *call = n->as.return_.values[0];
        int func_reg = alloc_temp(c);
        codegen_call_callee(c, call->as.call.callee, func_reg, call->line);
        uint16_t argc = 0;
        if (codegen_emit_args(c, call, func_reg, &argc)) return;
        const bool tailcall = call->as.call.callee->kind == AST_IDENT &&
                              find_func_sig(c, call->as.call.callee->as.ident.name) != NULL &&
                              c->protect_depth == 0;
        if (tailcall) {
            emit(c, OP_TAILCALL, func_reg, argc, RESULT_VARARGS, n->line);
            emit(c, OP_RETURN, func_reg, 0, RESULT_VARARGS, n->line);
        } else {
            codegen_emit_call(c, call->as.call.callee, func_reg, argc, RESULT_VARARGS, n->line);
            emit(c, OP_RETURN, func_reg, 0, RESULT_VARARGS, n->line);
        }
    } else if (nv > 0) {
        /* 预分配全部返回值槽：若某个返回值是调用（func_reg=alloc_temp 会复用
           reg_top 之下的槽），未预留的槽会被调用覆盖 → 前几个返回值丢失
           （`return a, b, c()` 曾把 b 覆盖成 c() 的结果）。 */
        int first = alloc_temp(c);
        for (size_t i = 1; i < nv; i++) alloc_temp(c);
        for (size_t i = 0; i < nv; i++) {
            codegen_expr(c, n->as.return_.values[i], first + (int)i);
        }
        if (nv == 1) emit(c, OP_RET1, first, 0, 1, n->line);   /* 单值专用返回 */
        else emit(c, OP_RETURN, first, 0, (uint16_t)nv, n->line);
    } else {
        emit(c, OP_RET0, 0, 0, 0, n->line);   /* 无值专用返回 */
    }
    restore_regs(c, mark);
}

static void codegen_func_decl_stmt(Codegen *c, const AstNode *n) {
    ObjClosure *f = codegen_func_value(c, n);
    if (c->has_error) return;
    const bool is_coro = body_has_yield(n->as.func.body);
    if (c->enclosing == NULL && c->scope_depth == 0) {
        int mark = c->reg_top;
        int t = alloc_temp(c);
        if (is_coro) {
            codegen_coroutine(c, n->as.func.body, f, t, n->line);
        } else {
            int k = add_const(c, obj_closure_val(f));
            emit(c, OP_CONST, t, k, 0, n->line);
        }
        int name = global_name(c, n->as.func.name);
        emit(c, OP_STORE_GLOBAL, t, name, 0, n->line);
        restore_regs(c, mark);
    } else {
        int slot = add_local(c, n->as.func.name, true);
        if (is_coro) {
            int mark = c->reg_top;
            int t = alloc_temp(c);
            codegen_coroutine(c, n->as.func.body, f, t, n->line);
            emit(c, OP_MOV, slot, t, 0, n->line);
            restore_regs(c, mark);
        } else {
            int k = add_const(c, obj_closure_val(f));
            emit(c, OP_CONST, slot, k, 0, n->line);
        }
    }
}

/* 选择性导入 `import Mod.{ member }` 由 codegen_import_scan 登记到 import_map（方案 A），
   引用发全限定 LOAD_GLOBAL；不再为每个成员建裸全局 STORE_GLOBAL（碰撞点）。 */

static void codegen_stmt(Codegen *c, const AstNode *n) {
    if (c->has_error) return;
    switch (n->kind) {
        case AST_VAR_DECL: codegen_var_decl(c, n); break;
        case AST_PROP_DECL: codegen_prop_decl(c, n); break;
        case AST_ASSIGN: codegen_assign(c, n); break;
        case AST_EXPR_STMT: codegen_expr_stmt(c, n); break;
        case AST_BLOCK: codegen_block_stmt(c, n); break;
        case AST_IF: codegen_if(c, n, -1); break;
        case AST_WHILE: codegen_while(c, n, -1); break;
        case AST_FOR_IN: codegen_for(c, n, -1); break;
        case AST_BREAK: codegen_break(c, n); break;
        case AST_CONTINUE: codegen_continue(c, n); break;
        case AST_RETURN: codegen_return(c, n); break;
        case AST_FUNC_DECL:
            if (n->as.func.name && c->enclosing == NULL && c->scope_depth == 0) {
                /* 顶层命名函数：pre-pass 已处理，主 pass 跳过 */
            } else {
                codegen_func_decl_stmt(c, n);
            }
            break;
        case AST_IMPORT:
            /* 方案 A：登记 import_map（整体模块别名 + 选择性成员 → 全限定名），不产生字节码；
               引用经 codegen_ident/codegen_member_get 发全限定 LOAD_GLOBAL，不再建裸全局。 */
            codegen_import_scan(c, n);
            break;
        case AST_EXPORT_MODULE:
            /* 模块加载由加载器负责，不产生字节码 */
            break;
        case AST_EXPORT:
            codegen_stmt(c, n->as.export_.decl);
            break;
        case AST_ERROR_SET_DECL:
            /* 编译期元信息：错误集在 pre-pass/本处注册，不产生字节码 */
            register_error_set(c, n);
            break;
        case AST_DEFER:
            codegen_defer(c, n);
            break;
        case AST_CLASS_DECL:
            codegen_nested_class(c, n);
            break;
        case AST_TRY:
        case AST_CATCH:
        case AST_MATCH: {
            int mark = c->reg_top;
            int t = alloc_temp(c);
            codegen_expr(c, n, t);
            restore_regs(c, mark);
            break;
        }
        case AST_THROW:
            codegen_throw(c, n);
            break;
        case AST_TRY_BLOCK:
            codegen_try_block(c, n);
            break;
        case AST_DO_CATCH:
            codegen_do_catch(c, n);
            break;
        default:
            error(c, n, "statement not supported in v1");
            break;
    }
}

/* ---------- 对外 API ---------- */

/* 顶层语句是否为"可执行语句"（模块禁止；入口以它或 main 作为程序入口） */
static bool is_executable_stmt(const AstNode *n) {
    switch (n->kind) {
        case AST_EXPR_STMT:
        case AST_ASSIGN:
        case AST_IF:
        case AST_WHILE:
        case AST_FOR_IN:
        case AST_BREAK:
        case AST_CONTINUE:
        case AST_RETURN:
        case AST_BLOCK:
        case AST_VAR_DECL:
        case AST_TRY_BLOCK:
        case AST_DO_CATCH:
            return true;
        default:
            return false;
    }
}

/* 共享编译核心。module_name != NULL 时以模块方式编译（内部全局加前缀），
   allow_main_call 时若无可执行顶层语句则自动调用 main()。 */
static ObjClosure *codegen_program_inner(Gc *gc, PeepholeState *ps, const AstNode *const *stmts, size_t n,
                                      CodegenError *err, const char *module_name,
                                      const char *const *internal_names, size_t n_internal,
                                      bool allow_main_call) {
    Codegen c = {0};
    c.gc = gc;
    c.peep = ps;
    c.enclosing = NULL;
    c.proto = proto_new();
    c.func_name = obj_string_new_permanent(gc, "<main>", 6);
    c.scope_depth = 0;
    c.reg_top = 2; /* slot0=函数值、slot1=self（入口函数为 null，恒保留） */
    c.max_regs = 2;
    c.module_prefix = module_name;
    c.err = err;
    for (size_t i = 0; i < n_internal; i++) name_set_add(&c.module_internal, internal_names[i]);

    /* 主函数捕获预扫描：顶层块局部可能被 lambda 捕获（顶层 var=全局，不装箱） */
    {
        NameSet scope = {0};
        for (size_t i = 0; i < n; i++) scan_stmt(stmts[i], &scope, &c.captured_names, 0, NULL);
        name_set_free(&scope);
    }

    /* 预扫描：import 的模块名（全名 + 末段短名）+ 选择性成员 → import_map。
       函数体/类方法在 pre-pass 3 就编译（早于主 pass 的 import 处理），须先记录
       imported_modules 使命名空间成员调用不注入 receiver，且压平映射须在编译函数体前就绪。 */
    for (size_t i = 0; i < n; i++) {
        const AstNode *s = stmts[i];
        const AstNode *d = s;
        while (d->kind == AST_EXPORT) d = d->as.export_.decl;
        if (d->kind == AST_IMPORT) codegen_import_scan(&c, d);
    }

    /* pre-pass 0：注册顶层计算属性名（let r => body）。顶层命名函数在 pre-pass 3 编译（早于主
       pass 的 codegen_prop_decl 注册），函数体内引用全局计算属性须在函数编译前知道它是 computed
       （读取时自动调用）——否则 LOAD_GLOBAL 拿到的闭包不被 OP_CALL，返回 <lambda>。 */
    for (size_t i = 0; i < n; i++) {
        const AstNode *s = stmts[i];
        const AstNode *d = s;
        while (d->kind == AST_EXPORT) d = d->as.export_.decl;
        if (d->kind == AST_PROP_DECL) {
            name_set_add(&c.computed_globals, d->as.prop_decl.name);
        }
    }

    /* pre-pass 1：错误集注册元信息 + 创建类对象 + 收集函数签名（支持前向引用/命名参数） */
    for (size_t i = 0; i < n; i++) {
        const AstNode *s = stmts[i];
        const AstNode *d = s;
        while (d->kind == AST_EXPORT) d = d->as.export_.decl;
        if (d->kind == AST_ERROR_SET_DECL) {
            register_error_set(&c, d);
        } else if (d->kind == AST_ENUM_DECL) {
            register_enum(&c, d);
        } else if (d->kind == AST_CLASS_DECL) {
            if (!d->as.class_.is_extern) codegen_class_create(&c, d, false); /* extern 类不建 ObjClass（运行时注入） */
        } else if (d->kind == AST_FUNC_DECL && d->as.func.name) {
            add_func_sig(&c, d->as.func.name, d->as.func.params, d->as.func.n_params,
                         d->as.func.has_varargs, d->as.func.is_throws);
        }
    }

    /* pre-pass 2：解析类继承链 */
    for (size_t i = 0; i < n; i++) {
        const AstNode *s = stmts[i];
        const AstNode *d = s;
        while (d->kind == AST_EXPORT) d = d->as.export_.decl;
        if (d->kind == AST_CLASS_DECL) {
            if (!d->as.class_.is_extern) {
                codegen_class_resolve_super(&c, d);
                if (c.has_error) goto fail;
            }
        }
    }

    /* pre-pass 3：顶层命名函数前置定义 + 编译类方法/默认值 + 存类全局 */
    for (size_t i = 0; i < n; i++) {
        const AstNode *s = stmts[i];
        const AstNode *d = s;
        while (d->kind == AST_EXPORT) d = d->as.export_.decl;
        if (d->kind == AST_FUNC_DECL && d->as.func.name) {
            codegen_func_decl_stmt(&c, d);
            if (c.has_error) goto fail;
        } else if (d->kind == AST_CLASS_DECL) {
            if (d->as.class_.is_extern) {
                /* extern 类：类对象由运行时在模块加载时注入，此处生成占位 null 全局
                   （让 build_module_namespace 能取到名字；真实值随后替换）。 */
                int t = alloc_temp(&c);
                emit(&c, OP_NULL, t, 0, 0, d->line); /* 占位 null：OP_NULL 免常量池 */
                int k = global_name(&c, d->as.class_.name);
                emit(&c, OP_STORE_GLOBAL, t, k, 0, d->line);
                c.reg_top--;
            } else {
                codegen_class_build(&c, d);
                if (c.has_error) goto fail;
                ObjClass *klass = find_class(&c, d->as.class_.name);
                int t = alloc_temp(&c);
                emit(&c, OP_CONST, t, add_const(&c, obj_class_val(klass)), 0, d->line);
                int k = global_name(&c, d->as.class_.name);
                emit(&c, OP_STORE_GLOBAL, t, k, 0, d->line);
                c.reg_top--;
            }
        } else if (d->kind == AST_ENUM_DECL) {
            codegen_enum_build(&c, d);
            if (c.has_error) goto fail;
        }
    }

    /* 主 pass */
    for (size_t i = 0; i < n; i++) {
        const AstNode *s = stmts[i];
        const AstNode *d = s;
        while (d->kind == AST_EXPORT) d = d->as.export_.decl;
        if (d->kind == AST_FUNC_DECL && d->as.func.name) continue;
        if (d->kind == AST_CLASS_DECL) continue;
        if (d->kind == AST_ENUM_DECL) continue;
        codegen_stmt(&c, s);
        if (c.has_error) goto fail;
    }

    /* 入口：无可执行顶层语句时，若存在 main() 则调用它 */
    if (allow_main_call) {
        bool has_exec = false, has_main = false;
        for (size_t i = 0; i < n; i++) {
            const AstNode *s = stmts[i];
            const AstNode *d = s;
            while (d->kind == AST_EXPORT) d = d->as.export_.decl;
            if (is_executable_stmt(d)) has_exec = true;
            if (d->kind == AST_FUNC_DECL && d->as.func.name && strcmp(d->as.func.name, "main") == 0) {
                has_main = true;
            }
        }
        if (!has_exec && has_main) {
            int t = alloc_temp(&c);
            alloc_temp(&c); /* t+1 = main 的 self 槽 */
            int k = global_ident(&c, "main");
            emit(&c, OP_LOAD_GLOBAL, t, k, 0, 0);
            emit(&c, OP_NULL, t + 1, 0, 0, 0); /* main 是普通函数：null self */
            emit(&c, OP_CALL, t, 1, RESULT_DISCARD, 0);
        }
    }

    /* 入口程序以 OP_EXIT 结束（VM 主循环 while(true)，EXIT 停止整个 vm_run）；
       模块 init 是普通函数，仍以 OP_RETURN 返回给加载器。 */
    if (allow_main_call) emit(&c, OP_EXIT, 0, 0, 0, 0);
    else emit(&c, OP_RET0, 0, 0, RESULT_DISCARD, 0);

    {
        c.proto->name = c.func_name;
        c.proto->arity = 1; /* 入口/模块 init 恒含 self（slot1=null） */
        c.proto->max_regs = (uint8_t)c.max_regs;
        finish_func(c.proto, c.peep);
        set_func_module(gc, c.module_prefix, c.proto);
        ObjClosure *f = obj_closure_new_permanent(gc, c.proto);
        name_set_free(&c.captured_names);
        name_set_free(&c.module_internal);
        name_set_free(&c.imported_modules);
        free_import_map(&c.import_map);
        free(c.capvals);
        free(c.locals);
        free(c.error_sets);
        free(c.enums);
        free(c.classes);
        free(c.func_sigs);
        free(c.var_classes);
        return f;
    }

fail:
    proto_free(c.proto);
    name_set_free(&c.captured_names);
    name_set_free(&c.module_internal);
    name_set_free(&c.imported_modules);  /* 成功路径在 return 前释放；失败路径曾漏（REPL 反复编译失败累积） */
    free_import_map(&c.import_map);
    free(c.capvals);
    free(c.locals);
    free(c.error_sets);
    free(c.enums);
    free(c.classes);
    free(c.func_sigs);
    free(c.var_classes);
    return NULL;
}

ObjClosure *codegen_program(Gc *gc, PeepholeState *ps, const AstNode *const *stmts, size_t n, CodegenError *err) {
    return codegen_program_inner(gc, ps, stmts, n, err, NULL, NULL, 0, true);
}

ObjClosure *codegen_program_module(Gc *gc, PeepholeState *ps, const AstNode *const *stmts, size_t n,
                                const char *module_name, const char *const *internal_names,
                                size_t n_internal, CodegenError *err) {
    return codegen_program_inner(gc, ps, stmts, n, err, module_name, internal_names, n_internal, false);
}
