#ifndef MICA_PROTO_H
#define MICA_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "opcodes.h"   /* Instr / OpCode / 指令编解码（2026-08-10 从本文件拆出） */
#include "value.h"
#include "obj.h"
#include "str.h"
#include "table.h"   /* ObjTable：索引化全局的稠密下标缓存 */

/* 函数原型：32 位定长指令（LuaJIT 式 ABC/AD 两格式，见 opcodes.h）+ 常量池 + 行号表。 */

/* 闭包捕获描述（OP_CLOSURE 的捕获值来源）：
   is_local=1：从当前帧的局部槽 index 建开放捕获值（指向该寄存器槽）；
   is_local=0：共享外层闭包 capvals[index]（传递捕获）。 */
typedef struct {
    uint8_t is_local;
    uint16_t index;
} CapvalDesc;

struct mica_context;   /* 前向声明（proto.h 不 include state.h，避免循环） */
struct CallFrame;

typedef struct Proto {
    /* ===== 热字段（VM 每帧执行必访；集中在前 48 字节 = 同一 cacheline）=====
       code/pool/三常量池/count 一次性载入，减少热路径 cache miss。 */
    Instr *code;             /* 指令流（pc 推进源） */
    void *pool;              /* 合并池基址（code+常量+标识符 单块内存） */
    Value *constants;        /* 通用常量池（OP_CONST） */
    Value *num_consts;       /* 数值常量池（OP_KNUM/ADDVN/GETI/ISEQN） */
    Value *str_consts;       /* 字符串常量池（OP_KSTR/ISEQS/ISNES） */
    size_t count;            /* 指令数 */
    /* ===== 次热：帧建立 / 错误行号 / 全局 / 成员 / 反射 ===== */
    int *lines;              /* 行号表（与 code 平行；仅运行期错误时读） */
    CapvalDesc *capvals;     /* 本函数捕获的外层变量描述（OP_CLOSURE） */
    String *ident_names;     /* 成员标识符池（GET/SET_MEMBER/GET_METHOD） */
    String *global_ident_names; /* 全局标识符池（LOAD_GLOBAL/STORE_GLOBAL/GLOAD_INDEX） */
    int32_t *global_entries; /* 索引化全局缓存：ident 下标 → 全局表稠密下标（-1=未定义；NULL=未解析） */
    struct CallSiteIC *ics;  /* 调用点内联缓存：GET_MEMBER 指令位置 → 成员解析（惰性 calloc size=count） */
    /* 标识符池去重哈希索引（仅编译/反序列化期使用，运行期不读）：
       槽存"池下标+1"（0=空）。历史实现是每加一个名字就线性扫全池 → 大量互不相同的
       名字下退化成 O(n²)（实测 20000 个不同全局名 415ms vs 重复名 40ms）。 */
    uint32_t *ident_htab;
    size_t ident_htab_cap;
    uint32_t *global_ident_htab;
    size_t global_ident_htab_cap;
    ObjString *name;         /* permanent 函数名（lambda 可为 NULL） */
    ObjString *module;       /* permanent 所属模块（System.IO 等），非模块函数 NULL */
    /* ===== 计数（uint32：上限受指令编码约束——全局标识符 65536、成员 512、寄存器 256，远小于 4G）===== */
    uint32_t n_capvals;      /* 捕获值数（OP_CLOSURE；ObjClosure.n_capvals 为 uint8，故 ≤255） */
    uint32_t n_constants;    /* 通用常量数 */
    uint32_t n_num_consts;   /* 数值常量数 */
    uint32_t n_str_consts;   /* 字符串常量数 */
    uint32_t n_ident_names;  /* 成员标识符数（≤ PROTO_MAX_CONSTS=512） */
    uint32_t n_global_ident_names; /* 全局标识符数（≤ PROTO_MAX_GLOBAL_IDENTS=65536） */
    /* ===== 小标量（函数元数据；ObjClosure 经 proto 读取，不随闭包携带）===== */
    uint8_t arity;           /* 声明参数个数（PROTO_MAX_REGS=256 上限内） */
    uint8_t max_regs;        /* 帧寄存器高水位（regalloc 回填） */
    bool is_throws;          /* 函数体含 throw：调用须被 try/catch 保护 */
    bool is_method;          /* 类方法（self 在签名中）：arity 含 self */
    bool is_coroutine;       /* 协程函数（体含 yield）：工厂是普通闭包 */
    /* ===== 编译期容量（仅 proto.c 构建期间读；uint32 压缩——容量远小于 4G，
       且 6 个 size_t(48B) → 6 个 uint32(24B) 每函数省 24 字节，
       10000 函数即省 240KB。运行期不读这些字段。）===== */
    uint32_t cap;                /* code 容量（Instr 数） */
    uint32_t cap_constants;      /* 通用常量区容量（Value 数） */
    uint32_t cap_num_consts;     /* 数值常量区容量 */
    uint32_t cap_str_consts;     /* 字符串常量区容量 */
    uint32_t cap_ident_names;    /* 成员标识符区容量（String 数） */
    uint32_t cap_global_ident_names; /* 全局标识符区容量 */
} Proto;

/* 调用点 IC 槽（D）：单态缓存。obj.field/obj.method() 热访问免 field_index 哈希 + 方法继承链。 */
typedef struct CallSiteIC {
    ObjClass *klass;      /* 缓存的实例类（permanent，无 GC 需求） */
    ObjClosure *method;   /* 方法闭包（kind==METHOD；经 klass 可达） */
    int16_t field_idx;    /* 字段绝对下标（kind==FIELD；-1=未用） */
    int16_t kind;         /* 0=空 1=字段 2=方法 */
} CallSiteIC;

Proto *proto_new(void);
void proto_free(Proto *ch);

/* 添加常量到通用池（任意类型：闭包/类/bool/null/match 字面量/IS_TYPE 名）。去重；返回下标。超限返回 -1。 */
int proto_add_const(Proto *ch, Value v);

/* 添加数值常量（num_consts 池，OP_KNUM/ADDVN/GETI/ISEQN 用）。去重；返回下标。超限返回 -1。 */
int proto_add_num_const(Proto *ch, Value v);

/* 添加字符串常量（str_consts 池，OP_KSTR/ISEQS/ISNES 用）。按内容去重；返回下标。超限返回 -1。 */
int proto_add_str_const(Proto *ch, Value v);

/* 添加标识符（按内容去重）；返回标识符池下标（成员名用）。 */
int proto_add_ident(Proto *ch, const char *name, size_t len);

/* 添加全局标识符（按内容去重，@ 归一）；返回全局标识符池下标（get_global 系列用）。
   容量 PROTO_MAX_GLOBAL_IDENTS=65536（LOAD_GLOBAL 的 d 16 位）。 */
int proto_add_global_ident(Proto *ch, const char *name, size_t len);

/* 保证合并池容量 ≥ need_code（字节码）/need_c（通用常量）/need_num（数值常量）/
   need_str（字符串常量）/need_i（标识符）/need_gi（全局标识符）；不足时整体 realloc
   （反序列化预分配用）。 */
void proto_reserve_pool(Proto *ch, size_t need_code, size_t need_c, size_t need_num,
                        size_t need_str, size_t need_i, size_t need_gi);

void proto_emit(Proto *ch, OpCode op, int a, int b, int c, int line);

size_t proto_code_len(const Proto *ch);
size_t proto_const_count(const Proto *ch);

/* 把第 instr_idx 条跳转的偏移写为 offset（相对该指令下一条）。 */
void proto_patch_jump(Proto *ch, size_t instr_idx, int offset);
int proto_line(const Proto *ch, size_t instr_idx);

const char *opcode_name(OpCode op);
void proto_disassemble(const Proto *ch, const char *name);
bool proto_validate(const Proto *ch);

#endif /* MICA_PROTO_H */
