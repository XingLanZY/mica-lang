/* 模块与程序集加载子系统（自 mica.c 拆出，2026-09-10 架构评审 A4）。
   职责：模块搜索路径解析 → 源文件解析 → 编译 → 初始化执行 → 命名空间构建；
   以及 .mlib 程序集/库的编译与动态加载。
   mica.c 保留：运行时生命周期、公开 API 门面、编译源与调用。
   ⚠️ 边界修正：迁出后 get_global_value/set_global_value/discard_frames/load_module 等
   为宿主层内部共享，须去掉 static（否则与 mica.c 的同名 static 冲突）。 */

#include "mica.h"

#include <assert.h>
#include <intrin.h>
#include <inttypes.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "assembly.h"
#include "builtin.h"
#include "codegen.h"
#include "gc.h"
#include "lexer.h"
#include "state.h"
#include "obj.h"
#include "parser.h"

/* 宿主 native（mica_native_fn）结果上限：staging 缓冲容量（旧 user_native_bridge 语义）。 */
#define MICA_NATIVE_MAX_RESULTS 8
#include "peephole.h"
#include "proto.h"
#include "libcoro.h"
#include "librefl.h"
#include "sema.h"
#include "table.h"
#include "value.h"
#include "vm.h"

#include "module_internal.h"

/* 顶层 `export module` 声明的模块名；没有返回 NULL。 */
static const char *module_header_name(const AstNode *const *stmts, size_t n) {
    for (size_t i = 0; i < n; i++) {
        if (stmts[i]->kind == AST_EXPORT_MODULE) return stmts[i]->as.export_module.name;
    }
    return NULL;
}

/* 校验入口：不能是模块；入口 = 顶层语句 XOR main()。返回 0=通过。 */
int validate_entry(const AstNode *const *stmts, size_t n, char *err, size_t err_cap) {
    if (module_header_name(stmts, n)) {
        snprintf(err, err_cap, "entry program cannot be a module (remove 'export module')");
        return 1;
    }
    bool has_exec = false, has_main = false;
    for (size_t i = 0; i < n; i++) {
        const AstNode *d = stmts[i];
        while (d->kind == AST_EXPORT) d = d->as.export_.decl;
        if (is_executable_stmt(d)) has_exec = true;
        if (d->kind == AST_FUNC_DECL && d->as.func.name && strcmp(d->as.func.name, "main") == 0) {
            has_main = true;
        }
    }
    if (has_exec && has_main) {
        snprintf(err, err_cap, "ambiguous entry: both top-level statements and main()");
        return 1;
    }
    if (!has_exec && !has_main) {
        snprintf(err, err_cap, "no entry point: top-level statements or main() required");
        return 1;
    }
    return 0;
}

/* 校验模块：必须有 export module；只允许声明/常量，禁止顶层语句与 var。 */
static int validate_module(const AstNode *const *stmts, size_t n, const char *name, char *err,
                           size_t err_cap) {
    const char *decl = module_header_name(stmts, n);
    if (!decl) {
        snprintf(err, err_cap, "module file must start with 'export module <name>'");
        return 1;
    }
    if (strcmp(decl, name) != 0) {
        snprintf(err, err_cap, "module name mismatch: file declares '%s' but imported as '%s'", decl,
                 name);
        return 1;
    }
    /* 注：System.* 保留给标准库的规则（@ 压缩的卫生）暂不在此硬检查——stdlib 自身
       也 declare System.Math，无法区分用户/标准模块；@ 压缩是双射（System.X → @X），
       无此规则也正确。将来可用"System 模块须来自 stdlib 搜索路径"实现。 */
    for (size_t i = 0; i < n; i++) {
        const AstNode *s = stmts[i];
        if (s->kind == AST_EXPORT_MODULE) continue;
        if (s->kind == AST_IMPORT) continue;
        const AstNode *d = s;
        while (d->kind == AST_EXPORT) d = d->as.export_.decl;
        if (d->kind == AST_VAR_DECL) {
            if (!d->as.var_decl.is_const) {
                snprintf(err, err_cap, "module '%s' cannot contain variables (use 'let')", name);
                return 1;
            }
            continue; /* let 常量是模块允许的声明 */
        }
        if (is_executable_stmt(d)) {
            snprintf(err, err_cap, "module '%s' cannot contain top-level statements", name);
            return 1;
        }
        switch (d->kind) {
            case AST_FUNC_DECL:
            case AST_CLASS_DECL:
            case AST_ERROR_SET_DECL:
            case AST_ENUM_DECL:
                break;
            default:
                snprintf(err, err_cap, "module '%s' may only declare functions/classes/error sets/"
                                       "enums or constants", name);
                return 1;
        }
    }
    return 0;
}

/* 收集模块内部声明名（func/let/class/error-set），供 codegen 前缀判定。 */
static void collect_internal_names(const AstNode *const *stmts, size_t n, const char ***out,
                                   size_t *out_n) {
    size_t cap = 0;
    for (size_t i = 0; i < n; i++) {
        const AstNode *d = stmts[i];
        while (d->kind == AST_EXPORT) d = d->as.export_.decl;
        const char *ename = NULL;
        switch (d->kind) {
            case AST_FUNC_DECL: ename = d->as.func.name; break;
            case AST_CLASS_DECL: ename = d->as.class_.name; break;
            case AST_ERROR_SET_DECL: ename = d->as.error_set.name; break;
            case AST_ENUM_DECL: ename = d->as.enum_.name; break;
            case AST_IMPORT: /* 选择性导入成员：作为模块内部名（module.member 作用域） */
                for (size_t j = 0; j < d->as.import.n_names; j++) {
                    list_add(out, out_n, &cap, d->as.import.names[j]);
                }
                continue;
            case AST_VAR_DECL:
                if (d->as.var_decl.n_names > 0) ename = d->as.var_decl.names[0];
                break;
            default: break;
        }
        if (ename) list_add(out, out_n, &cap, ename);
    }
}

/* 收集顶层 import 名。 */
void collect_import_names(const AstNode *const *stmts, size_t n, const char ***out,
                                 size_t *out_n) {
    size_t cap = 0;
    for (size_t i = 0; i < n; i++) {
        if (stmts[i]->kind == AST_IMPORT) {
            list_add(out, out_n, &cap, stmts[i]->as.import.name);
        }
    }
}

static bool loaded_or_loading(mica_runtime *rt, const char *name) {
    for (size_t i = 0; i < rt->n_loaded; i++) {
        if (strcmp(rt->loaded_modules[i], name) == 0) return true;
    }
    return false;
}

int load_module(mica_runtime *rt, const char *name);
static int load_assembly_core(mica_runtime *rt, const char *path, ObjClosure **out_entry);

/* 解析模块文件路径：C# 命名空间风格——每个模块单独一个点分文件名文件，
   `import System.IO` → <search_path>/System.IO.mica（不按目录分层）。
   优先源码 .mica，其次程序集库 .mlib。 */
static char *resolve_module_file(mica_runtime *rt, const char *name) {
    char path[1024];
    for (size_t i = 0; i < rt->n_search; i++) {
        snprintf(path, sizeof(path), "%s/%s.mica", rt->search_path[i], name);
        if (file_exists(path)) return dup_str(path);
        snprintf(path, sizeof(path), "%s/%s.mlib", rt->search_path[i], name);
        if (file_exists(path)) return dup_str(path);
    }
    return NULL;
}

/* 丢弃当前所有帧（复用 context 前）：关闭并释放各帧的开放捕获值数组。
   错误路径 vm_run 会留下未弹出的帧（捕获值数组未释放），下次运行前必须清理，否则泄漏。 */
void discard_frames(mica_context *ctx) {
    for (FrameBlock *b = ctx->frame_blocks; b; b = b->next) {
        for (int i = 0; i < b->used; i++) {
            obj_frame_close_capvals(ctx->gc, ctx, &b->frames[i]);
        }
        b->used = 0;
    }
    ctx->frame_count = 0;
    ctx->cur_block = ctx->frame_blocks; /* 空栈：回落头部块，下次压帧复用块 0 */
}

/* 运行一个已编译函数（临时 context，共享 runtime 的 gc 与 globals）。 */
static int run_program(mica_runtime *rt, ObjClosure *f, char *err, size_t err_cap) {
    mica_context *ctx = mica_context_new(rt);
    
    CTX_CLR_ERROR(ctx);
    str_clear(&ctx->st_error_msg);
    ctx->st_error_value = val_null();
    ctx->frame_count = 0;
    ctx->stack_top = ctx->stack;
    ctx->defer_top = 0;
    CallFrame *f0 = ctx_frame_new(ctx);
    *f0 = (CallFrame){ .closure = f, .savedpc = f->proto->code, .u = {0}, .base = ctx->stack,
                       .want = RESULT_DISCARD };
    ctx->stack[0] = obj_closure_val(f);
    ctx->stack[1] = val_null(); /* 统一约定：slot1=null self（入口/模块 init 普通函数） */
    ctx->stack_top = ctx->stack + (int)f->proto->max_regs;
    const CoroResult r = vm_run(ctx, NULL, 0);
    const int rc = (r == CORO_ERROR) ? 1 : 0;
    if (rc != 0 && err) snprintf(err, err_cap, "%s", ctx->st_error_msg ? ctx->st_error_msg : "");
    mica_context_free(ctx);
    return rc;
}

bool get_global_value(mica_runtime *rt, const char *name, Value *out) {
    char buf[512];
    size_t len = strlen(name);
    if (len < sizeof(buf)) {
        len = str_compress_system(name, len, buf); /* System.* → @*（运行时规范扁平键） */
        name = buf;
    }
    return objtable_get_kv(rt->globals, fnv_hash(name, len), name, len, out);
}

void set_global_value(mica_runtime *rt, const char *name, Value v) {
    char buf[512];
    size_t len = strlen(name);
    if (len < sizeof(buf)) {
        len = str_compress_system(name, len, buf); /* System.* → @*（运行时规范扁平键） */
        name = buf;
    }
    objtable_set_kv(&rt->gc, rt->globals, fnv_hash(name, len), name, len, v);
}

/* 把模块命名空间注册为嵌套全局，使成员链引用可用：
   "Math" → globals["Math"]=ns；"System.IO" → globals["System"]["IO"]=ns。
   点分名额外把末段绑定为顶层短全局（import System.IO 后 IO.x 也可用）。 */
void set_nested_namespace(mica_runtime *rt, const char *name, ObjMap *ns) {
    char buf[512];
    snprintf(buf, sizeof(buf), "%s", name);
    char *seg = buf;
    ObjMap *cur = NULL;
    for (;;) {
        char *dot = strchr(seg, '.');
        if (!dot) {
            ObjString *k = obj_string_new_permanent(&rt->gc, seg, strlen(seg));
            if (cur) obj_map_set(&rt->gc, cur, obj_string_val(k), obj_map_val(ns));
            else set_global_value(rt, seg, obj_map_val(ns));
            break;
        }
        *dot = '\0';
        ObjString *k = obj_string_new_permanent(&rt->gc, seg, strlen(seg));
        Value existing = val_null();
        ObjMap *next = NULL;
        if (cur) {
            if (obj_map_get(cur, obj_string_val(k), &existing) && val_is_obj(existing) &&
                val_as_obj(existing)->type == OBJ_MAP) {
                next = (ObjMap *)val_as_obj(existing);
            } else {
                next = obj_map_new_permanent(&rt->gc); /* 命名空间中间 map 长期存活 */
                obj_map_set(&rt->gc, cur, obj_string_val(k), obj_map_val(next));
            }
        } else {
            if (get_global_value(rt, seg, &existing) && val_is_obj(existing) &&
                val_as_obj(existing)->type == OBJ_MAP) {
                next = (ObjMap *)val_as_obj(existing);
            } else {
                next = obj_map_new_permanent(&rt->gc);
                set_global_value(rt, seg, obj_map_val(next));
            }
        }
        cur = next;
        seg = dot + 1;
    }
    /* 点分名的全限定扁平全局：globals["System.Math"] = ns（方案 A：import 压平 `Math` →
       "System.Math" 直引用；**不再建裸末段别名 globals["Math"]**——那是跨程序集/跨导入碰撞点）。
       单段名已在上方设置；嵌套 globals["System"]["Math"] 保留（反射/全限定遍历用）。 */
    if (strchr(name, '.')) {
        set_global_value(rt, name, obj_map_val(ns));
    }
}

/* 收集模块的导出名（仅带 export 修饰的声明），供命名空间重建。 */
static void collect_export_names(const AstNode *const *stmts, size_t n, const char ***out,
                                 size_t *out_n) {
    const char **arr = NULL;
    size_t cnt = 0, cap = 0;
    for (size_t i = 0; i < n; i++) {
        const AstNode *s = stmts[i];
        if (s->kind != AST_EXPORT) continue;
        const AstNode *d = s->as.export_.decl;
        const char *ename = NULL;
        switch (d->kind) {
            case AST_FUNC_DECL: ename = d->as.func.name; break;
            case AST_CLASS_DECL: ename = d->as.class_.name; break;
            case AST_ERROR_SET_DECL: ename = d->as.error_set.name; break;
            case AST_ENUM_DECL: ename = d->as.enum_.name; break;
            case AST_VAR_DECL:
                if (d->as.var_decl.is_const && d->as.var_decl.n_names > 0) {
                    ename = d->as.var_decl.names[0];
                }
                break;
            default: break;
        }
        if (!ename) continue;
        list_add(&arr, &cnt, &cap, ename);
    }
    *out = arr;
    *out_n = cnt;
}

/* extern 类：模块 init 把占位 null 全局替换为运行时注入的真实类对象
   （WeakRef → System.Runtime；反射类 → System.Reflection）。 */
static void inject_extern_classes(mica_runtime *rt, const AstNode *const *stmts, size_t n,
                                  const char *module_name) {
    for (size_t i = 0; i < n; i++) {
        const AstNode *s = stmts[i];
        if (s->kind != AST_EXPORT) continue;
        const AstNode *d = s->as.export_.decl;
        if (d->kind != AST_CLASS_DECL || !d->as.class_.is_extern) continue;
        /* 反射类（System.Reflection）由 librefl 经注册 API 注入，此处只处理 WeakRef */
        if (strcmp(d->as.class_.name, "WeakRef") != 0) continue;
        char flat[512];
        snprintf(flat, sizeof(flat), "%s.%s", module_name, d->as.class_.name);
        set_global_value(rt, flat, obj_class_val(rt->weakref_class));
    }
}

/* 从初始化后的扁平全局构建模块命名空间（仅 export 的声明），并注册嵌套全局。 */
static void build_module_namespace(mica_runtime *rt, const char *module_name,
                                   const AstNode *const *stmts, size_t n) {
    ObjMap *ns = obj_map_new_permanent(&rt->gc); /* permanent：命名空间长期存活，纯分代 minor 不回收 */
    for (size_t i = 0; i < n; i++) {
        const AstNode *s = stmts[i];
        if (s->kind != AST_EXPORT) continue;
        const AstNode *d = s->as.export_.decl;
        const char *ename = NULL;
        switch (d->kind) {
            case AST_FUNC_DECL: ename = d->as.func.name; break;
            case AST_CLASS_DECL: ename = d->as.class_.name; break;
            case AST_ERROR_SET_DECL: ename = d->as.error_set.name; break;
            case AST_ENUM_DECL: ename = d->as.enum_.name; break;
            case AST_VAR_DECL:
                if (d->as.var_decl.is_const && d->as.var_decl.n_names > 0) {
                    ename = d->as.var_decl.names[0];
                }
                break;
            default: break;
        }
        if (!ename) continue;
        char flat[512];
        snprintf(flat, sizeof(flat), "%s.%s", module_name, ename);
        Value v;
        if (get_global_value(rt, flat, &v)) {
            ObjString *k = obj_string_new_permanent(&rt->gc, ename, strlen(ename));
            obj_map_set(&rt->gc, ns, obj_string_val(k), v);
        }
    }
    set_nested_namespace(rt, module_name, ns);
}

/* 解析源码（AST 生命周期受 parser 约束），返回 stmts/n，调用方负责 parser/lexer 释放。
   出错时 err 填消息，out_line/out_col 填首个错误的位置（可为 NULL）。 */
int parse_source(const char *src, size_t len, Parser **out_p, Lexer **out_lx,
                        AstNode ***out_stmts, size_t *out_n, char *err, size_t err_cap,
                        int *out_line, int *out_col) {
    Lexer *lx = lexer_new(src, len);
    if (!lx) {
        snprintf(err, err_cap, "out of memory");
        if (out_line) *out_line = 0;
        if (out_col) *out_col = 0;
        return 1;
    }
    Parser *p = parser_new(lx);
    size_t n = 0;
    AstNode **stmts = parser_parse(p, &n);
    if (parser_error_count(p) > 0) {
        const ParserError *e = parser_error(p, 0);
        snprintf(err, err_cap, "%s", e->message);
        if (out_line) *out_line = e->line;
        if (out_col) *out_col = e->col;
        parser_free(p);
        lexer_free(lx);
        return 1;
    }
    *out_p = p;
    *out_lx = lx;
    *out_stmts = stmts;
    *out_n = n;
    return 0;
}

/* 注册 runtime 现有全局为 sema 外部全局。 */
int register_external_global(const String *key, Value value, void *user) {
    (void)value;
    sema_add_external_global((Sema *)user, str_data(key));
    return 0;
}

int load_module(mica_runtime *rt, const char *name) {
    if (loaded_or_loading(rt, name)) return 0;

    char *path = resolve_module_file(rt, name);
    if (!path) {
        str_set(&rt->last_error, "module '%s' not found in search path", name);
        str_clear(&rt->last_error_path);
        rt->last_error_line = 0;
        return 1;
    }
    /* 程序集库（.mlib）：直接加载，不依赖源码 */
    {
        const size_t plen = strlen(path);
        if (plen > 5 && strcmp(path + plen - 5, ".mlib") == 0) {
            const int rc = load_assembly_core(rt, path, NULL);
            free(path);
            if (rc == 0) str_clear(&rt->last_error_path);
            return rc;
        }
    }
    str_set(&rt->last_error_path, "%s", path);
    rt->last_error_line = 0;

    size_t len = 0;
    char *src = read_file(path, &len);
    free(path);
    if (!src) {
        str_set(&rt->last_error, "cannot read module '%s'", name);
        return 1;
    }

    Parser *p = NULL;
    Lexer *lx = NULL;
    AstNode **stmts = NULL;
    size_t n = 0;
    int pline = 0, pcol = 0;
    {
        char ebuf[512];
        if (parse_source(src, len, &p, &lx, &stmts, &n, ebuf, sizeof(ebuf), &pline, &pcol) != 0) {
            str_set(&rt->last_error, "%s", ebuf);
            rt->last_error_line = pline;
            (void)pcol;
            free(src);
            return 1;
        }
    }
    free(src);

    {
        char vbuf[512];
        if (validate_module(stmts, n, name, vbuf, sizeof(vbuf)) != 0) {
            str_set(&rt->last_error, "%s", vbuf);
            parser_free(p);
            lexer_free(lx);
            return 1;
        }
    }

    /* 循环导入检测：先标记加载中 */
    list_add(&rt->loaded_modules, &rt->n_loaded, &rt->cap_loaded, dup_str(name));

    /* 递归加载本模块的 import */
    const char **imports = NULL;
    size_t n_imports = 0;
    collect_import_names(stmts, n, &imports, &n_imports);
    for (size_t i = 0; i < n_imports; i++) {
        if (load_module(rt, imports[i]) != 0) {
            unmark_module(rt, name);
            free(imports);
            parser_free(p);
            lexer_free(lx);
            return 1;
        }
    }
    free(imports);

    /* 模块内部声明名 → codegen 前缀判定 */
    const char **internal = NULL;
    size_t n_internal = 0;
    collect_internal_names(stmts, n, &internal, &n_internal);

    /* sema：外部全局 = runtime 现有全局（含已加载模块命名空间） */
    Sema *sema = sema_new();
    objtable_iter(rt->globals, register_external_global, sema);
    sema_analyze(sema, stmts, n);
    if (sema_error_count(sema) > 0) {
        const ParserError *e = sema_error(sema, 0);
        str_set(&rt->last_error, "%s", e->message);
        rt->last_error_line = e->line;
        unmark_module(rt, name);
        sema_free(sema);
        free(internal);
        parser_free(p);
        lexer_free(lx);
        return 1;
    }
    sema_free(sema);

    CodegenError cerr = {0};
    ObjClosure *init = codegen_program_module(&rt->gc, &rt->peep, (const AstNode *const *)stmts, n, name, internal,
                                           n_internal, &cerr);
    free(internal);
    if (!init) {
        str_set(&rt->last_error, "%s", cerr.message);
        rt->last_error_line = cerr.line;
        unmark_module(rt, name);
        parser_free(p);
        lexer_free(lx);
        return 1;
    }

    {
        char ebuf[512];
        if (run_program(rt, init, ebuf, sizeof(ebuf)) != 0) {
            str_set(&rt->last_error, "%s", ebuf);
            unmark_module(rt, name);
            parser_free(p);
            lexer_free(lx);
            return 1;
        }
    }
    /* extern 类：把占位 null 全局替换为运行时注入的真实类对象（WeakRef 等） */
    inject_extern_classes(rt, stmts, n, name);
    /* 命名空间须在 parser arena 释放前构建（stmts 仍有效） */
    build_module_namespace(rt, name, stmts, n);
    /* 保存 init 函数与导出名：程序集编译/命名空间重建用（permanent，GC 不回收） */
    {
        ObjString *mk = obj_string_new_permanent(&rt->gc, name, strlen(name));
        objtable_set_kv(&rt->gc, rt->module_inits, obj_string_hash(mk), mk->chars, mk->len,
                        obj_closure_val(init));
        const char **exports = NULL;
        size_t n_exports = 0;
        collect_export_names(stmts, n, &exports, &n_exports);
        ObjArray *ea = obj_array_new(&rt->gc, n_exports);
        for (size_t i = 0; i < n_exports; i++) {
            obj_array_push(&rt->gc, ea,
                           obj_string_val(obj_string_new_permanent(&rt->gc, exports[i], strlen(exports[i]))));
        }
        free(exports);
        objtable_set_kv(&rt->gc, rt->module_exports, obj_string_hash(mk), mk->chars, mk->len, obj_array_val(ea));
    }
    parser_free(p);
    lexer_free(lx);
    str_clear(&rt->last_error_path); /* 成功加载则清除 */
    return 0;
}

int mica_load_module(mica_runtime *rt, const char *name) {
    str_clear(&rt->last_error);
    return load_module(rt, name);
}

/* ---------- 程序集（assembly） ---------- */

/* 重置已加载状态，保证收集到的是本程序集的模块。 */
static void reset_loaded_modules(mica_runtime *rt) {
    for (size_t i = 0; i < rt->n_loaded; i++) free((void *)rt->loaded_modules[i]);
    rt->n_loaded = 0;
    /* 旧表永久对象：显式释放（摘链+finalize），否则永久链作根会让上一程序集的内容永不回收 */
    gc_free_permanent(&rt->gc, objtable_as_obj(rt->module_inits));
    gc_free_permanent(&rt->gc, objtable_as_obj(rt->module_exports));
    rt->module_inits = objtable_new_permanent(&rt->gc);
    rt->module_exports = objtable_new_permanent(&rt->gc);
    /* 新表必须重新登记为显式根：explicit_roots 模式下未登记的 permanent 表不参与标记，
       而这两张表持**运行期**对象（module_exports 持 obj_array_new 建的导出名数组、
       module_inits 持运行期闭包）→ 不登记会被下一次 minor 误回收（实测导出数组的
       count 被复用内存覆写成别的值）。旧表的根已由 gc_free_permanent 摘除。 */
    gc_add_root(&rt->gc, objtable_as_obj(rt->module_inits));
    gc_add_root(&rt->gc, objtable_as_obj(rt->module_exports));
}

/* 收集当前已加载模块（按依赖序）的名字 / 导出名 / init 函数。
   out_exports 是 char****（写回一个 char*** 数组：每个模块的导出名列表）。 */
static void collect_loaded_modules(mica_runtime *rt, const char ***out_names, size_t *out_n,
                                   char ****out_exports, size_t **out_counts,
                                   ObjClosure ***out_inits) {
    const size_t n = rt->n_loaded;
    const char **names = malloc((n ? n : 1) * sizeof(char *));
    ObjClosure **inits = malloc((n ? n : 1) * sizeof(ObjClosure *));
    char ***exports = malloc((n ? n : 1) * sizeof(char **));
    size_t *export_counts = malloc((n ? n : 1) * sizeof(size_t));
    for (size_t i = 0; i < n; i++) {
        names[i] = rt->loaded_modules[i];
        ObjString *k = obj_string_new_permanent(&rt->gc, rt->loaded_modules[i],
                                                strlen(rt->loaded_modules[i]));
        Value v;
        objtable_get_kv(rt->module_inits, obj_string_hash(k), k->chars, k->len, &v);
        inits[i] = (val_is_obj(v) && val_as_obj(v)->type == OBJ_CLOSURE)
                       ? (ObjClosure *)val_as_obj(v)
                       : NULL;
        Value ev;
        objtable_get_kv(rt->module_exports, obj_string_hash(k), k->chars, k->len, &ev);
        if (val_is_obj(ev) && val_as_obj(ev)->type == OBJ_ARRAY) {
            ObjArray *ea = (ObjArray *)val_as_obj(ev);
            export_counts[i] = ea->count;
            exports[i] = malloc((ea->count ? ea->count : 1) * sizeof(char *));
            for (size_t j = 0; j < ea->count; j++) {
                ObjString *s = (ObjString *)val_as_obj(obj_array_get(ea, j));
                exports[i][j] = s->chars;
            }
        } else {
            export_counts[i] = 0;
            exports[i] = NULL;
        }
    }
    *out_names = names;
    *out_n = n;
    *out_exports = exports;
    *out_counts = export_counts;
    *out_inits = inits;
}

static void free_collected_modules(size_t n, char **names, char ***exports, size_t *counts,
                                   ObjClosure **inits) {
    /* 注意：names[i] 指向 rt->loaded_modules[i]（runtime 拥有），exports[i][j]
       指向 ObjString->chars（GC 拥有）——都不 free，只释放我们自己 malloc 的数组。 */
    (void)names;
    for (size_t i = 0; i < n; i++) free(exports[i]);
    free(names);
    free(exports);
    free(counts);
    free(inits);
}

/* 编译程序集：编译入口（含递归 import 模块），把各模块 init + 入口序列化为单个二进制。
   entry_path 所在目录自动加入模块搜索路径（mica_dofile 行为）。 */
int mica_compile_assembly(mica_runtime *rt, const char *entry_path, const char *out_path) {
    reset_loaded_modules(rt);
    librefl_init(rt); /* 重注册注入模块（System.Reflection）：reset 清了 loaded_modules */
    libcoro_init(rt); /* 重注册协程模块（coroutine 命名空间） */

    mica_context *ctx = mica_context_new(rt);
    if (mica_dofile(ctx, entry_path) != 0) {
        str_set(&rt->last_error, "%s", mica_error_message(ctx));
        mica_context_free(ctx);
        return 1;
    }

    const char **names;
    size_t n;
    char ***exports;
    size_t *export_counts;
    ObjClosure **inits;
    collect_loaded_modules(rt, &names, &n, &exports, &export_counts, &inits);
    const int rc = assembly_write(&rt->gc, out_path, names, n, exports, export_counts, inits,
                                  ctx->program);
    free_collected_modules(n, (char **)names, exports, export_counts, inits);
    mica_context_free(ctx);
    return rc;
}

/* 编译库：把单个模块文件（export module）编译为 .mlib，供其它代码 import。
   module_path 所在目录自动加入模块搜索路径（供其递归 import）。 */
int mica_compile_library(mica_runtime *rt, const char *module_path, const char *out_path) {
    /* 模块名 = 文件名去扩展（System.IO.mica → System.IO） */
    char name[512];
    const char *base = strrchr(module_path, '/');
    const char *p = base ? base + 1 : module_path;
    size_t plen = strlen(p);
    if (plen > 5 && strcmp(p + plen - 5, ".mica") == 0) plen -= 5;
    else if (plen > 5 && strcmp(p + plen - 5, ".mlib") == 0) plen -= 5;
    if (plen >= sizeof(name)) plen = sizeof(name) - 1;
    memcpy(name, p, plen);
    name[plen] = '\0';

    reset_loaded_modules(rt);
    librefl_init(rt); /* 重注册注入模块（System.Reflection）：reset 清了 loaded_modules */
    libcoro_init(rt); /* 重注册协程模块（coroutine 命名空间） */
    {
        char dir[1024];
        snprintf(dir, sizeof(dir), "%s", module_path);
        char *slash = strrchr(dir, '/');
        if (slash) {
            *slash = '\0';
            add_search_path_front(rt, dir);
        } else {
            add_search_path_front(rt, ".");
        }
    }

    if (load_module(rt, name) != 0) return 1;

    const char **names;
    size_t n;
    char ***exports;
    size_t *export_counts;
    ObjClosure **inits;
    collect_loaded_modules(rt, &names, &n, &exports, &export_counts, &inits);
    const int rc = assembly_write(&rt->gc, out_path, names, n, exports, export_counts, inits, NULL);
    free_collected_modules(n, (char **)names, exports, export_counts, inits);
    return rc;
}

/* 加载程序集的所有模块：反序列化 + 按依赖序运行模块 init（注册命名空间）+ 填充已加载表。
   out_entry 返回入口（库程序集为 NULL）。错误填 rt->last_error。 */
static int load_assembly_core(mica_runtime *rt, const char *path, ObjClosure **out_entry) {
    char **names = NULL;
    size_t n = 0;
    char ***exports = NULL;
    size_t *export_counts = NULL;
    ObjClosure **inits = NULL;
    ObjClosure *entry = NULL;
    if (assembly_read(&rt->gc, path, &names, &n, &exports, &export_counts, &inits, &entry) != 0) {
        str_set(&rt->last_error, "cannot load assembly '%s'", path);
        return 1;
    }
    for (size_t i = 0; i < n; i++) {
        if (names[i] && loaded_or_loading(rt, names[i])) continue; /* 重复加载（Assembly.load 二次）跳过 */
        char err[256] = {0};
        if (run_program(rt, inits[i], err, sizeof(err)) != 0) {
            str_set(&rt->last_error, "module '%s' init failed: %s",
                    names[i] ? names[i] : "?", err);
            for (size_t j = 0; j < n; j++) free(names[j]);
            free(names);
            for (size_t j = 0; j < n; j++) {
                for (size_t k = 0; k < export_counts[j]; k++) free(exports[j][k]);
                free(exports[j]);
            }
            free(exports);
            free(export_counts);
            free(inits);
            return 1;
        }
        /* 从扁平全局重建命名空间 map（与 build_module_namespace 等价） */
        ObjMap *ns = obj_map_new(&rt->gc);
        for (size_t j = 0; j < export_counts[i]; j++) {
            char flat[512];
            snprintf(flat, sizeof(flat), "%s.%s", names[i], exports[i][j]);
            Value v;
            if (get_global_value(rt, flat, &v)) {
                obj_map_set(&rt->gc, ns,
                            obj_string_val(obj_string_new_permanent(&rt->gc, exports[i][j],
                                                                    strlen(exports[i][j]))),
                            v);
            }
        }
        set_nested_namespace(rt, names[i], ns);
        ObjString *k = obj_string_new_permanent(&rt->gc, names[i], strlen(names[i]));
        objtable_set_kv(&rt->gc, rt->module_inits, obj_string_hash(k), k->chars, k->len,
                        obj_closure_val(inits[i]));
        list_add(&rt->loaded_modules, &rt->n_loaded, &rt->cap_loaded, dup_str(names[i]));
    }
    for (size_t j = 0; j < n; j++) free(names[j]);
    free(names);
    for (size_t j = 0; j < n; j++) {
        for (size_t k = 0; k < export_counts[j]; k++) free(exports[j][k]);
        free(exports[j]);
    }
    free(exports);
    free(export_counts);
    free(inits);
    if (out_entry) *out_entry = entry;
    return 0;
}

/* 加载程序集：反序列化重建所有模块 init + 入口，按依赖序执行模块 init（注册命名空间），
   随后 mica_run 可运行入口。 */
int mica_load_assembly(mica_context *ctx, const char *path) {
    ObjClosure *entry = NULL;
    if (load_assembly_core(ctx->rt, path, &entry) != 0) {
        str_set(&ctx->st_error_msg, "%s", mica_runtime_error(ctx->rt));
        ctx->st_error_line = ctx->rt->last_error_line;
        return 1;
    }
    /* 程序集名 = 文件基名（去扩展名），供反射 Assembly.name */
    {
        const char *base = path;
        const char *slash = strrchr(path, '/');
        const char *bs = strrchr(path, '\\');
        const char *sep = (bs && (!slash || bs > slash)) ? bs : slash;
        if (sep) base = sep + 1;
        const char *dot = strrchr(base, '.');
        size_t blen = dot ? (size_t)(dot - base) : strlen(base);
        if (blen == 0) blen = strlen(base);
        str_set(&ctx->rt->assembly_name, "%.*s", (int)blen, base);
    }
    ctx->program = entry;
    str_clear(&ctx->st_error_msg);
    ctx->st_error_line = 0;
    return 0;
}

/* 运行时动态加载库程序集（.mlib）：反序列化 + 按依赖序运行模块 init + 注册命名空间。
   加载的模块成为运行时的一部分（可从反射 Assembly 访问）；不改变当前程序集名。
   重复加载同一模块由 load_assembly_core 跳过。 */
int mica_runtime_load_library(mica_runtime *rt, const char *path) {
    str_clear(&rt->last_error);
    if (load_assembly_core(rt, path, NULL) != 0) return 1;
    str_clear(&rt->last_error_path);
    return 0;
}
