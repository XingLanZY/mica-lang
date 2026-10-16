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
#include "module_internal.h" /* 模块子系统（module.c）共享声明 */

/* ============================================================
   模块架构（高内聚、低耦合）：
   - mica.h       对外 C API（唯一公开头文件，不透明类型 mica_runtime/mica_context/
                  mica_value；宿主只需包含它）。
   - state.h  内部结构定义：struct mica_runtime / struct mica_context（mica.c /
                  vm.c / builtin.c / assembly.c 共享字段）。mica_context = 一个执行线程/协程。
   - mica.c       引擎核心：mica_runtime/mica_context 生命周期、模块加载器、程序集、
                  编译/运行驱动、对外 API。
   - vm.c         寄存器 VM 执行引擎：vm_run（纯函数，操作 mica_context 字段）、全部 op
                  处理器、defer/协程辅助。只依赖 mica.h + vm.h + state.h。
   - vm.h         VM 共享工具原型（runtime_error/make_error/value_to_cstr）+ vm_run。
   - builtin.c    内置类型成员方法（array/string/map）+ System.Builtin 标准库底层
                  native。独立模块：只依赖 vm.h/obj.h/value.h/gc.h/table.h + mica.h +
                  state.h。
   扩展点（添加新功能不影响旧功能）：
   - 新对象类型：obj.h 枚举 + obj.c 工厂 + obj_vtables_init 注册，gc.c 零改动。
   - 新指令：proto.h 枚举 + codegen.c 发射 + vm.c 的 vm_run switch 一臂。
   - 新内置方法：builtin.c 方法表加一行。
   - 新语法：parser.c 语句/表达式分支 + sema.c 校验 + codegen.c 生成。
   ============================================================ */

/* 内部结构定义在 state.h（mica.c / vm.c / builtin.c / assembly.c 共享）。 */

/* 动态字符串字段辅助（定义见 dup_str 后）：错误消息/路径字段 malloc 动态串。 */
static void str_vset(char **dst, const char *fmt, va_list ap);
void str_set(char **dst, const char *fmt, ...);
void str_clear(char **dst);

/* 内部：错误核心（行号由显式 pc 定位；pc 为空或越界则行号为 0）。 */
static void runtime_error_pc(mica_context *ctx, CallFrame *fr, const Instr *pc, const char *fmt,
                             va_list ap) {
    if (CTX_HAS_ERROR(ctx)) return;
    CTX_SET_ERROR(ctx);
    str_vset(&ctx->st_error_msg, fmt, ap);
    ctx->st_error_line = (fr && pc && pc > fr->closure->proto->code)
                             ? proto_line(fr->closure->proto, (int)(pc - fr->closure->proto->code - 1))
                             : 0;
    /* 在 vm_run 执行期内（active_jmp 有效）：longjmp 直接展开到最近 vm_run 的 setjmp，
       跳过中间 C 帧（错误即终止）。active_jmp 为 NULL（mica_call 直调 native
       无外层 vm_run）：保持标志式返回，调用方按 has_error 传播。 */
    if (ctx->active_jmp) longjmp(*ctx->active_jmp, 1);
}

/* 运行期错误：设置当前 context 的 error（fr 可为 NULL，仅用于行号定位）。
   行号取自 fr->savedpc——调用方须保证其指向"当前指令之后"（帧边界/解释器已写回）。 */
void runtime_error(mica_context *ctx, CallFrame *fr, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    runtime_error_pc(ctx, fr, fr ? fr->savedpc : NULL, fmt, ap);
    va_end(ap);
}

/* 运行期错误（显式 pc 版）：行号直接算自 pc，不依赖 fr->savedpc。
   vm_run 主循环热路径用局部 pc 直接报错，免写回 fr->savedpc。 */
void runtime_error_at(mica_context *ctx, CallFrame *fr, const Instr *pc, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    runtime_error_pc(ctx, fr, pc, fmt, ap);
    va_end(ap);
}

/* 设置 context 运行期错误消息（无 longjmp；协程引擎预检错误等"以值返回"路径用，
   调用方决定如何把错误转成错误值/状态码）。 */
void ctx_set_error(mica_context *ctx, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    str_vset(&ctx->st_error_msg, fmt, ap);
    va_end(ap);
    CTX_SET_ERROR(ctx);
}

/* 内部：按给定栈大小创建 context（主程序用 MICA_STACK_SIZE；协程用 MICA_CORO_STACK_INIT）。
   context 是 GC 对象（OBJ_CONTEXT）：栈/defer 经 vtable trace，finalize 释放缓冲。
   is_main：主 context 用 gc_alloc_permanent（permanent 链作根，宿主持有，
   mica_context_free 显式释放）；协程 context 用 gc_alloc（运行期对象，GC contexts 链保底）。 */
mica_context *mica_context_new_sized(mica_runtime *rt, size_t stack_size, bool is_main) {
    mica_context *ctx = is_main
        ? gc_alloc_permanent(&rt->gc, sizeof(mica_context), OBJ_CONTEXT)
        : gc_alloc(&rt->gc, sizeof(mica_context), OBJ_CONTEXT);
    ctx->rt = rt;
    ctx->gc = &rt->gc;
    ctx->stack = calloc(stack_size, sizeof(Value)); /* 未用槽零值（=VAL_NULL），可被 GC 安全扫描 */
    assert(ctx->stack != NULL);
    ctx->stack_size = (int)stack_size;
    ctx->frame_blocks = NULL; /* 帧块链表：惰性分配（首帧压入时建块 0） */
    ctx->cur_block = NULL;
    ctx->defers = malloc(MICA_DEFER_STACK * sizeof(Value)); /* 初始 defer 容量，可增长 */
    assert(ctx->defers != NULL);
    ctx->defer_cap = MICA_DEFER_STACK;
    /* gc_alloc 不再 memset：显式初始化所有依赖 0 的字段（openupval/frame_count/stack.top 等）。 */
    ctx->ctx_next = NULL;
    ctx->program = NULL;
    ctx->error_path = NULL;
    ctx->stack_top = ctx->stack;
    ctx->opencapval = NULL;
    ctx->frame_count = 0;
    ctx->status = CORO_NONE; /* 主 context 态；协程由 coroutine.create 置 CORO_SUSPENDED */
    ctx->st_error_msg = NULL;
    ctx->st_error_line = 0;
    ctx->st_error_value = val_null();
    ctx->active_jmp = NULL;
    ctx->coro_closure = NULL;
    ctx->coro_start_args = NULL;
    ctx->n_coro_start_args = 0;
    ctx->resume_depth = 0;
    ctx->yield_requested = false;
    ctx->yield_value = val_null();
    ctx->yield_cont = NULL;
    ctx->yield_ud = NULL;
    ctx->defer_top = 0;
    /* 栈不再注册为 GC 根范围：per-context trace（context_trace）扫活动帧的寄存器范围
       [base, base+max_regs) + openupval 链。闲置 context（无帧）不扫栈，
       避免整栈扫描（协程/复用期间的空栈区）。openupval 链由 gc_alloc 零初始化为 NULL。 */
    /* 协程 context（is_main=false）挂入 GC contexts 链：middle/old 协程栈值每轮重扫
       （resume 可能无屏障写入 young）。主 context 是 permanent（permanent 链每轮作根），不入链。
       D2：主 context 登记为显式根（持有编译程序/栈值等运行期引用）。 */
    if (is_main) {
        gc_add_root(&rt->gc, (Obj *)ctx);
    } else {
        gc_add_context(&rt->gc, (Obj *)ctx);
    }
    return ctx;
}

/* 主程序 Value 栈大小见 vm.h（MICA_STACK_SIZE）。 */

/* ---------- mica_value ↔ Value 互转（内联实现见 value.h val_wrap/val_unwrap） ---------- */

static mica_value wrap(Value v) { return val_wrap(v); }

static Value unwrap(mica_value mv) { return val_unwrap(mv); }


/* mica_runtime / mica_context 结构定义在 state.h（mica.c/vm.c/builtin.c/assembly.c 共享）。 */

/* ---------- 内部工具 ---------- */

char *dup_str(const char *s) {
    size_t n = strlen(s);
    char *p = malloc(n + 1);
    assert(p != NULL);
    memcpy(p, s, n + 1);
    return p;
}

/* 动态字符串字段辅助：malloc 新值并释放旧值。错误消息/路径字段不用大内联数组，
   减小 mica_runtime/mica_context 结构体足迹（错误路径非热路径，malloc 开销可忽略）。 */
static void str_vset(char **dst, const char *fmt, va_list ap) {
    char buf[1024];
    vsnprintf(buf, sizeof(buf), fmt, ap); /* 先格式化（可能引用 *dst 旧值），后 free */
    free(*dst);
    *dst = dup_str(buf);
}
void str_set(char **dst, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    str_vset(dst, fmt, ap);
    va_end(ap);
}
void str_clear(char **dst) {
    free(*dst);
    *dst = NULL;
}

char *read_file(const char *path, size_t *out_len) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return NULL;
    }
    long sz = ftell(f);
    if (sz < 0) {
        fclose(f);
        return NULL;
    }
    rewind(f);
    char *buf = malloc((size_t)sz + 1);
    if (!buf) {
        fclose(f);
        return NULL;
    }
    size_t rd = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    buf[rd] = '\0';
    *out_len = rd;
    return buf;
}

bool file_exists(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    fclose(f);
    return true;
}

void list_add(const char ***arr, size_t *n, size_t *cap, const char *item) {
    if (*n == *cap) {
        size_t ncap = *cap ? *cap * 2 : 8;
        const char **na = realloc((void *)*arr, ncap * sizeof(const char *));
        assert(na != NULL);
        *arr = na;
        *cap = ncap;
    }
    (*arr)[(*n)++] = item;
}

/* 从 loaded_modules 摘除指定模块名（加载失败回滚用；保持剩余顺序不变）。
   否则失败后名字残留 → 重试命中 loaded_or_loading 静默跳过，模块永远不加载。 */
void unmark_module(mica_runtime *rt, const char *name) {
    for (size_t i = 0; i < rt->n_loaded; i++) {
        if (strcmp(rt->loaded_modules[i], name) == 0) {
            free((void *)rt->loaded_modules[i]);
            for (size_t j = i; j + 1 < rt->n_loaded; j++)
                rt->loaded_modules[j] = rt->loaded_modules[j + 1];
            rt->n_loaded--;
            return;
        }
    }
}

Value make_error(mica_context *ctx, const char *msg) {
    /* 构造窗口：obj_error_new 内部 gc_alloc 可能触发 minor，而 s 只被 C 局部引用 →
       会被误回收（e->message 悬垂）。用 unsafe 窗口包住这一对分配。 */
    gc_begin_unsafe(ctx->gc);
    ObjString *s = obj_string_new(ctx->gc, msg, strlen(msg));
    Value e = obj_error_val(obj_error_new(ctx->gc, s, NULL, NULL, NULL, NULL, 0));
    gc_end_unsafe(ctx->gc);
    return e;
}

/* ---------- 宿主 native 注册（ObjNativeClosure 直存 mica_native_fn，无桥接） ---------- */

bool get_global_value(mica_runtime *rt, const char *name, Value *out);
void set_global_value(mica_runtime *rt, const char *name, Value v);
void set_nested_namespace(mica_runtime *rt, const char *name, ObjMap *ns);

void mica_register_native(mica_runtime *rt, const char *name, mica_native_fn fn, uint32_t arity) {
    ObjNativeClosure *n = obj_native_closure_new_permanent(&rt->gc, fn, name, arity);
    set_global_value(rt, name, obj_native_closure_val(n));
}

/* ---------- 模块实体注册（宿主/原生模块注入） ---------- */

/* 取或建模块命名空间 map（"System.Reflection" → globals["System"]["Reflection"]=ObjMap）。
   沿点分路径解析现有嵌套 map（set_nested_namespace 链接的是嵌套路径，无扁平键），
   未找到则新建并链接。 */
static ObjMap *module_namespace_for(mica_runtime *rt, const char *module) {
    char buf[512];
    snprintf(buf, sizeof(buf), "%s", module);
    char *seg = buf;
    Value cur = val_null();
    for (;;) {
        char *dot = strchr(seg, '.');
        if (dot) *dot = '\0';
        if (val_is_null(cur)) {
            if (!get_global_value(rt, seg, &cur)) break;
        } else {
            if (!val_is_obj(cur) || val_as_obj(cur)->type != OBJ_MAP) break;
            ObjString *k = obj_string_new_permanent(&rt->gc, seg, strlen(seg));
            if (!obj_map_get((ObjMap *)val_as_obj(cur), obj_string_val(k), &cur)) break;
        }
        if (!dot) {
            if (val_is_obj(cur) && val_as_obj(cur)->type == OBJ_MAP) return (ObjMap *)val_as_obj(cur);
            break;
        }
        seg = dot + 1;
    }
    ObjMap *ns = obj_map_new_permanent(&rt->gc);
    set_nested_namespace(rt, module, ns);
    return ns;
}

/* 把实体注册进模块：设置扁平全局 "module.name" + 加入命名空间 map。 */
static void module_set_member(mica_runtime *rt, const char *module, const char *name, Value v) {
    char flat[512];
    snprintf(flat, sizeof(flat), "%s.%s", module, name);
    set_global_value(rt, flat, v);
    ObjMap *ns = module_namespace_for(rt, module);
    ObjString *k = obj_string_new_permanent(&rt->gc, name, strlen(name));
    obj_map_set(&rt->gc, ns, obj_string_val(k), v);
}

void mica_register_module(mica_runtime *rt, const char *module) {
    /* 标记已加载 + 建命名空间：import module 命中 loaded_or_loading 跳过源码；
       asm.modules()/types() 会把它计入模块枚举（System.Reflection 是真实模块）。 */
    for (size_t i = 0; i < rt->n_loaded; i++)
        if (strcmp(rt->loaded_modules[i], module) == 0) return; /* 已注册 */
    list_add(&rt->loaded_modules, &rt->n_loaded, &rt->cap_loaded, dup_str(module));
    module_namespace_for(rt, module);
}

mica_value mica_register_class(mica_runtime *rt, const char *module, const char *name) {
    mica_register_module(rt, module);
    char flat[512];
    snprintf(flat, sizeof(flat), "%s.%s", module, name);
    Value existing;
    if (get_global_value(rt, flat, &existing) && val_is_obj(existing) &&
        val_as_obj(existing)->type == OBJ_CLASS) {
        return wrap(existing); /* 幂等：已注册过（如 reset_loaded_modules 后重注册）则复用 */
    }
    ObjString *mn = obj_string_new_permanent(&rt->gc, name, strlen(name));
    ObjClass *klass = obj_class_new_permanent(&rt->gc, mn);
    klass->module = obj_string_new_permanent(&rt->gc, module, strlen(module));
    module_set_member(rt, module, name, obj_class_val(klass));
    return wrap(obj_class_val(klass));
}

/* 注册原生类（QuickJS JS_NewClass 类似物）：建 ObjNativeClass + 存 globals["module.name"]。
   def->name 是类名（挂 module 命名空间）；def->userdata_size 是每实例 C 结构体字节；
   def->finalizer 是实例回收回调（可为 NULL）。幂等：同名已注册返回已有类。 */
mica_value mica_new_native_class(mica_runtime *rt, const char *module,
                                 const mica_native_class_def *def) {
    mica_register_module(rt, module);
    char flat[512];
    snprintf(flat, sizeof(flat), "%s.%s", module, def->name);
    Value existing;
    if (get_global_value(rt, flat, &existing) && val_is_obj(existing) &&
        val_as_obj(existing)->type == OBJ_NATIVE_CLASS) {
        return wrap(existing); /* 幂等 */
    }
    ObjString *mn = obj_string_new_permanent(&rt->gc, def->name, strlen(def->name));
    ObjString *mmod = obj_string_new_permanent(&rt->gc, module, strlen(module));
    ObjNativeClass *nc = obj_native_class_new_permanent(&rt->gc, mn, mmod, def->userdata_size,
                                                        def->finalizer);
    module_set_member(rt, module, def->name, obj_native_class_val(nc));
    return wrap(obj_native_class_val(nc));
}

/* 创建原生实例（QuickJS JS_NewObjectClass 类似物）：userdata 零初始化。 */
mica_value mica_new_native_instance(mica_context *ctx, mica_value klass) {
    Value k = unwrap(klass);
    if (!val_is_obj(k) || val_as_obj(k)->type != OBJ_NATIVE_CLASS) return mica_null();
    ObjNativeInstance *i = obj_native_instance_new(ctx->gc, (ObjNativeClass *)val_as_obj(k));
    return wrap(obj_native_instance_val(i));
}

void *mica_native_userdata(mica_context *ctx, mica_value obj, size_t *size) {
    (void)ctx;
    Value v = unwrap(obj);
    if (!val_is_obj(v) || val_as_obj(v)->type != OBJ_NATIVE_INSTANCE) return NULL;
    ObjNativeInstance *i = (ObjNativeInstance *)val_as_obj(v);
    if (size) *size = i->klass->userdata_size;
    return i->userdata;
}

void mica_register_field(mica_runtime *rt, mica_value klass, const char *name) {
    Value k = unwrap(klass);
    if (!val_is_obj(k) || val_as_obj(k)->type != OBJ_CLASS) return;
    ObjClass *cl = (ObjClass *)val_as_obj(k);
    /* 追加声明字段 + 重建 field_index（继承前缀布局；须在任何实例创建前调用） */
    cl->field_names = realloc(cl->field_names, (cl->n_field_names + 1) * sizeof(ObjString *));
    assert(cl->field_names != NULL);
    cl->field_names[cl->n_field_names] = obj_string_new_permanent(&rt->gc, name, strlen(name));
    cl->n_field_names++;
    cl->field_index = NULL; /* 失效重建 */
    cl->field_cache_valid = false;
    obj_class_build_field_index(&rt->gc, cl);
}

void mica_register_method(mica_runtime *rt, mica_value klass, const char *name,
                          mica_native_fn fn, uint32_t arity, bool is_static) {
    Value k = unwrap(klass);
    if (!val_is_obj(k)) return;
    Obj *ko = val_as_obj(k);
    if (ko->type == OBJ_CLASS) {
        ObjClass *cl = (ObjClass *)ko;
        ObjNativeClosure *n = obj_native_closure_new_permanent(&rt->gc, fn, name, arity);
        n->is_method = true; /* 方法 native：receiver@slot1、实参从 slot2、arity 含 receiver */
        ObjTable *tbl = is_static ? cl->static_methods : cl->methods;
        objtable_set_kv(&rt->gc, tbl, fnv_hash(name, strlen(name)), name, strlen(name),
                        obj_native_closure_val(n));
    } else if (ko->type == OBJ_NATIVE_CLASS) {
        ObjNativeClass *nc = (ObjNativeClass *)ko;
        ObjNativeClosure *n = obj_native_closure_new_permanent(&rt->gc, fn, name, arity);
        n->is_method = true;
        ObjTable *tbl = is_static ? nc->static_methods : nc->methods;
        objtable_set_kv(&rt->gc, tbl, fnv_hash(name, strlen(name)), name, strlen(name),
                        obj_native_closure_val(n));
    }
}

void mica_register_property(mica_runtime *rt, mica_value klass, const char *name,
                            mica_native_fn getter, mica_native_fn setter) {
    Value k = unwrap(klass);
    if (!val_is_obj(k) || val_as_obj(k)->type != OBJ_CLASS) return;
    ObjClass *cl = (ObjClass *)val_as_obj(k);
    const uint32_t h = fnv_hash(name, strlen(name));
    if (getter) {
        ObjNativeClosure *g = obj_native_closure_new_permanent(&rt->gc, getter, name, 1);
        g->is_method = true; /* 属性 getter：receiver 作首参 */
        objtable_set_kv(&rt->gc, cl->getters, h, name, strlen(name), obj_native_closure_val(g));
        cl->has_getters = true;
    }
    if (setter) {
        ObjNativeClosure *s = obj_native_closure_new_permanent(&rt->gc, setter, name, 2);
        s->is_method = true; /* 属性 setter：receiver + value */
        objtable_set_kv(&rt->gc, cl->setters, h, name, strlen(name), obj_native_closure_val(s));
        cl->has_setters = true;
    }
}

void mica_register_function(mica_runtime *rt, const char *module, const char *name,
                            mica_native_fn fn, uint32_t arity) {
    mica_register_module(rt, module);
    ObjNativeClosure *n = obj_native_closure_new_permanent(&rt->gc, fn, name, arity);
    module_set_member(rt, module, name, obj_native_closure_val(n));
}

void mica_register_constant(mica_runtime *rt, const char *module, const char *name, mica_value v) {
    mica_register_module(rt, module);
    module_set_member(rt, module, name, unwrap(v));
}

void mica_register_enum(mica_runtime *rt, const char *module, const char *name,
                        const char *const *members, const int64_t *values, size_t n) {
    mica_register_module(rt, module);
    ObjMap *m = obj_map_new_permanent(&rt->gc);
    for (size_t i = 0; i < n; i++) {
        ObjString *k = obj_string_new_permanent(&rt->gc, members[i], strlen(members[i]));
        const int64_t v = values ? values[i] : (int64_t)i;
        obj_map_set(&rt->gc, m, obj_string_val(k), val_float((double)v));
    }
    module_set_member(rt, module, name, obj_map_val(m));
}

mica_runtime *mica_context_runtime(mica_context *ctx) {
    return ctx->rt;
}

/* 执行已编译函数（0=正常，1=运行期错误；error_msg/line 已填）。 */
/* 运行状态 st 直到：帧数降到 stop_count（返回 CORO_DONE 且 *out=首结果）、
   协程 yield（返回 CORO_YIELDED 且 *out=产出值）、或运行期错误（CORO_ERROR）。
   调用方负责设置初始帧与错误复位。 */
/* ---------- 运行时 ---------- */

mica_runtime *mica_runtime_new(void) {
    mica_runtime *rt = calloc(1, sizeof(mica_runtime));
    assert(rt != NULL);
    Gc *gc = &rt->gc; /* 内嵌 Gc 值 */
    gc_init(gc);
    obj_vtables_init(gc);
    table_vtables_init(gc);
    ObjTable *globals = objtable_new_permanent(gc);
    install_natives(gc, globals);
    rt->globals = globals;
    rt->module_inits = objtable_new_permanent(gc);
    rt->module_exports = objtable_new_permanent(gc);
    /* GC_ULTRA_PLAN D2 显式根集：globals / module_inits / module_exports 是 permanent 表，
       持有**运行期**值（全局变量、导出名 ObjArray、init ObjClosure、模块命名空间 map——
       命名空间经 set_nested_namespace 链入 globals，随 globals 覆盖）。登记为显式根后，
       minor/播种只 trace 根集，protos/类/永久字符串（permanent 链其余对象）免每轮 trace。
       ⚠️ 新增"permanent 持运行期引用"的对象必须 gc_add_root（MICA_GC_DEBUG 自动校验）。 */
    gc_add_root(gc, (Obj *)rt->globals);
    gc_add_root(gc, (Obj *)rt->module_inits);
    gc_add_root(gc, (Obj *)rt->module_exports);
    rt->weakref_class = mica_weakref_class(gc);
    librefl_init(rt); /* 反射模块：注册 System.Reflection 命名空间 + 类 */
    libcoro_init(rt); /* 协程模块：注册 coroutine 命名空间（create/resume/cancel/status） */
    str_set(&rt->assembly_name, "%s", "main"); /* 默认程序集名 */
    peephole_state_init(&rt->peep, true); /* 内置规则 + 默认开（按 runtime 隔离） */
    const char *peep_env = getenv("MICA_PEEPHOLE");
    if (peep_env && peep_env[0] == '0') peephole_set_enabled(&rt->peep, false);
    const char *gc_env = getenv("MICA_GC");
    if (gc_env && gc_env[0] == '0') gc_set_enabled(gc, false); /* 诊断：关 GC 测分配裸开销 */
    /* D2：生产启用显式根集（根已登记；MICA_GC_ROOTS=0 回退"trace 全 permanent 链"）。 */
    const char *roots_env = getenv("MICA_GC_ROOTS");
    if (!(roots_env && roots_env[0] == '0')) gc_set_explicit_roots(gc, true);
    return rt;
}

void mica_runtime_free(mica_runtime *rt) {
    if (!rt) return;
    for (size_t i = 0; i < rt->n_search; i++) free((void *)rt->search_path[i]);
    free(rt->search_path);
    for (size_t i = 0; i < rt->n_loaded; i++) free((void *)rt->loaded_modules[i]);
    free(rt->loaded_modules);
    free(rt->assembly_name);
    free(rt->last_error);
    free(rt->last_error_path);
    gc_deinit(&rt->gc);
    free(rt);
}

/* 窥孔优化开关：进程级（codegen 阶段运行，无 runtime 引用）。 */
void mica_set_peephole_enabled(mica_runtime *rt, bool enabled) {
    if (rt) peephole_set_enabled(&rt->peep, enabled);
}

void mica_runtime_add_search_path(mica_runtime *rt, const char *dir) {
    list_add(&rt->search_path, &rt->n_search, &rt->cap_search, dup_str(dir));
}

/* 前插搜索路径：入口/库所在目录应优先于全局 stdlib，允许用户模块覆盖标准库。 */
void add_search_path_front(mica_runtime *rt, const char *dir) {
    if (rt->n_search == rt->cap_search) {
        size_t ncap = rt->cap_search ? rt->cap_search * 2 : 8;
        rt->search_path = realloc(rt->search_path, ncap * sizeof(char *));
        assert(rt->search_path != NULL);
        rt->cap_search = ncap;
    }
    memmove(rt->search_path + 1, rt->search_path, rt->n_search * sizeof(char *));
    rt->search_path[0] = dup_str(dir);
    rt->n_search++;
}

const char *mica_runtime_error(const mica_runtime *rt) {
    return rt->last_error ? rt->last_error : "";
}

void mica_runtime_set_global(mica_runtime *rt, const char *name, mica_value v) {
    char buf[512];
    size_t len = strlen(name);
    if (len < sizeof(buf)) {
        len = str_compress_system(name, len, buf); /* System.* → @*（运行时规范扁平键） */
        name = buf;
    }
    objtable_set_kv(&rt->gc, rt->globals, fnv_hash(name, len), name, len, unwrap(v));
}

mica_value mica_runtime_get_global(mica_runtime *rt, const char *name) {
    char buf[512];
    size_t len = strlen(name);
    if (len < sizeof(buf)) {
        len = str_compress_system(name, len, buf); /* System.* → @*（运行时规范扁平键） */
        name = buf;
    }
    Value v;
    mica_value out = wrap(val_null());
    if (objtable_get_kv(rt->globals, fnv_hash(name, len), name, len, &v)) out = wrap(v);
    return out;
}

/* ---------- 模块加载器 ---------- */

/* 顶层语句是否"可执行"（模块禁止；入口用它判断是否有语句入口） */
bool is_executable_stmt(const AstNode *n) {
    switch (n->kind) {
        case AST_EXPR_STMT:
        case AST_ASSIGN:
        case AST_IF:
        case AST_WHILE:
        case AST_FOR_IN:
        case AST_BREAK:
        case AST_CONTINUE:
        case AST_RETURN:
        case AST_THROW:
        case AST_BLOCK:
        case AST_VAR_DECL:
        case AST_TRY_BLOCK:
        case AST_DO_CATCH:
            return true;
        default:
            return false;
    }
}


/* ---------- 上下文 ---------- */

mica_context *mica_context_new(mica_runtime *rt) {
    /* 主 context 是 permanent 对象：入 permanent 链即被 GC 作根 trace，宿主持有。 */
    return mica_context_new_sized(rt, MICA_STACK_SIZE, true);
}

void mica_context_free(mica_context *ctx) {
    if (!ctx) return;
    /* 主 context 是 permanent 对象：从 permanent 链摘除后显式 finalize+free
       （permanent 永不回收，宿主释放即销毁，不等 GC）。
       宿主总是先 mica_context_free 再 mica_runtime_free，无双重释放。 */
    gc_free_permanent(ctx->gc, (Obj *)ctx);
}

/* 完整前端（入口）：lex → parse → 校验 → 加载 import → sema → codegen。
   出错填充 ctx->st_error_msg/line。 */
static int compile_source(mica_context *ctx, const char *src, size_t len) {
    Parser *p = NULL;
    Lexer *lx = NULL;
    AstNode **stmts = NULL;
    size_t n = 0;
    int pline = 0, pcol = 0;
    {
        char ebuf[512];
        if (parse_source(src, len, &p, &lx, &stmts, &n, ebuf, sizeof(ebuf), &pline, &pcol) != 0) {
            str_set(&ctx->st_error_msg, "%s", ebuf);
            ctx->st_error_line = pline;
            (void)pcol;
            return 1;
        }
    }
    str_clear(&ctx->error_path);

    /* 入口校验：不能是模块；入口 = 顶层语句 XOR main() */
    {
        char vbuf[512];
        if (validate_entry(stmts, n, vbuf, sizeof(vbuf)) != 0) {
            str_set(&ctx->st_error_msg, "%s", vbuf);
            parser_free(p);
            lexer_free(lx);
            return 1;
        }
    }

    /* 递归加载顶层 import（模块初始化会注册其命名空间全局） */
    const char **imports = NULL;
    size_t n_imports = 0;
    collect_import_names(stmts, n, &imports, &n_imports);
    for (size_t i = 0; i < n_imports; i++) {
        if (load_module(ctx->rt, imports[i]) != 0) {
            str_set(&ctx->st_error_msg, "%s", mica_runtime_error(ctx->rt));
            ctx->st_error_line = ctx->rt->last_error_line;
            str_set(&ctx->error_path, "%s", mica_runtime_error_path(ctx->rt));
            free(imports);
            parser_free(p);
            lexer_free(lx);
            return 1;
        }
    }
    free(imports);

    Sema *sema = sema_new();
    objtable_iter(ctx->rt->globals, register_external_global, sema);
    sema_analyze(sema, stmts, n);
    if (sema_error_count(sema) > 0) {
        const ParserError *e = sema_error(sema, 0);
        str_set(&ctx->st_error_msg, "%s", e->message);
        ctx->st_error_line = e->line;
        sema_free(sema);
        parser_free(p);
        lexer_free(lx);
        return 1;
    }
    sema_free(sema);

    CodegenError cerr = {0};
    ObjClosure *f = codegen_program(&ctx->rt->gc, &ctx->rt->peep, (const AstNode *const *)stmts, n, &cerr);
    parser_free(p);
    lexer_free(lx);
    if (!f) {
        str_set(&ctx->st_error_msg, "%s", cerr.message);
        ctx->st_error_line = cerr.line;
        return 1;
    }

    ctx->program = f;
    str_clear(&ctx->st_error_msg);
    ctx->st_error_line = 0;
    return 0;
}

int mica_dofile(mica_context *ctx, const char *path) {
    size_t len = 0;
    char *src = read_file(path, &len);
    if (!src) {
        str_set(&ctx->st_error_msg, "cannot open '%s'", path);
        ctx->st_error_line = 0;
        return 1;
    }
    /* 程序集名 = 入口基名（去目录与扩展名），供反射 Assembly.name */
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
    /* 默认把入口文件所在目录加入模块搜索路径 */
    {
        char dir[1024];
        const char *slash = strrchr(path, '/');
        const char *bs = strrchr(path, '\\');
        const char *sep = (bs && (!slash || bs > slash)) ? bs : slash;
        if (sep) {
            size_t dlen = (size_t)(sep - path);
            memcpy(dir, path, dlen);
            dir[dlen] = '\0';
            add_search_path_front(ctx->rt, dir);
        } else {
            add_search_path_front(ctx->rt, ".");
        }
    }
    const int rc = compile_source(ctx, src, len);
    free(src);
    if (rc != 0) return 1;
    return mica_run(ctx);
}

int mica_compile(mica_context *ctx, const char *src, size_t len) {
    return compile_source(ctx, src, len);
}

int mica_run(mica_context *ctx) {
    if (!ctx->program) {
        str_set(&ctx->st_error_msg, "%s", "no program compiled");
        ctx->st_error_line = 0;
        return 1;
    }

    CTX_CLR_ERROR(ctx);
    str_clear(&ctx->st_error_msg);
    ctx->st_error_value = val_null();
    ctx->st_error_line = 0;
    discard_frames(ctx);
    ctx->stack_top = ctx->stack;
    ctx->defer_top = 0;
    CallFrame *f0 = ctx_frame_new(ctx);
    *f0 = (CallFrame){ .closure = ctx->program, .savedpc = ctx->program->proto->code, .u = {0},
                       .base = ctx->stack, .want = RESULT_DISCARD };
    ctx->stack[0] = obj_closure_val(ctx->program);
    ctx->stack[1] = val_null(); /* 统一约定：slot1=null self（入口普通函数） */
    ctx->stack_top = ctx->stack + (int)ctx->program->proto->max_regs;
    const CoroResult r = vm_run(ctx, NULL, 0);
    const int rc = (r == CORO_ERROR) ? 1 : 0;
    if (rc == 0) {
        str_clear(&ctx->st_error_msg);
        ctx->st_error_line = 0;
    } else {
        str_set(&ctx->st_error_msg, "%s", ctx->st_error_msg ? ctx->st_error_msg : "");
        ctx->st_error_line = ctx->st_error_line;
    }
    return rc;
}

int mica_dostring(mica_context *ctx, const char *src, size_t len) {
    if (mica_compile(ctx, src, len) != 0) return 1;
    return mica_run(ctx);
}

int mica_call(mica_context *ctx, const char *name, const mica_value *args, int argc,
              mica_value *result) {
    Value fn;
    if (!get_global_value(ctx->rt, name, &fn)) {
        str_set(&ctx->st_error_msg, "no such function '%s'", name);
        ctx->st_error_line = 0;
        return 1;
    }

    CTX_CLR_ERROR(ctx);
    str_clear(&ctx->st_error_msg);
    ctx->st_error_value = val_null();
    ctx->st_error_line = 0;
    discard_frames(ctx);
    ctx->stack_top = ctx->stack;
    ctx->defer_top = 0;
    /* 容量检查：栈只在 VM 路径经 ensure_stack 增长，宿主直调必须自己保证 2+argc 槽可用
       （旧实现直接写 stack[2+i]，argc > 65533 时越界写）。 */
    if (!vm_ensure_stack(ctx, ctx->stack + 2 + (argc > 0 ? argc : 0))) {
        str_set(&ctx->st_error_msg, "stack overflow");
        ctx->st_error_line = 0;
        return 1;
    }
    ctx->stack[0] = fn;         /* 被调函数也必须可被 GC 看到（native 可能分配） */
    ctx->stack[1] = val_null(); /* 统一约定：slot1=null self */
    for (int i = 0; i < argc; i++) ctx->stack[2 + i] = unwrap(args[i]);

    Value result_v = val_null();
    int rc = 0;
    if (val_is_obj(fn) && val_as_obj(fn)->type == OBJ_NATIVE_CLOSURE) {
        ObjNativeClosure *nat = (ObjNativeClosure *)val_as_obj(fn);
        /* native 直调时 frame_count == 0：context_trace 对无帧 context 会扫
           [stack, stack_top)（已按此修正），故把栈顶抬到实参范围末尾，
           使 slot0（函数）/slot1（self）/实参全部进入根集——否则 native 内部分配触发
           minor 会把这些值回收（旧实现只有栈、无根）。 */
        ctx->stack_top = ctx->stack + 2 + argc;
        /* 结果 staging：宿主 native 的 results 独立于实参（mica_call 实参就在 stack+2，
           结果也写 stack+2 会覆盖未消费实参；多返回值同理）。上限 MICA_NATIVE_MAX_RESULTS。 */
        mica_value nres[MICA_NATIVE_MAX_RESULTS];
        const uint32_t n = nat->func(ctx, (mica_value *)(void *)(ctx->stack + 2),
                                     (uint32_t)argc, nres);
        const uint32_t cn = n > MICA_NATIVE_MAX_RESULTS ? MICA_NATIVE_MAX_RESULTS : n;
        /* 结果写入前先把栈顶括到结果范围（写 nres[i] 期间可能不再分配，但结果值本身
           在返回宿主后仍须短暂可根——这里抬 top 覆盖之）。 */
        ctx->stack_top = ctx->stack + (2 + argc > 2 + (int)cn ? 2 + argc : 2 + (int)cn);
        for (uint32_t i = 0; i < cn; i++) ctx->stack[2 + i] = val_unwrap(nres[i]);
        if (cn > 0) result_v = ctx->stack[2];
        ctx->stack_top = ctx->stack + 2 + (int)cn;
    } else if (val_is_obj(fn) && val_as_obj(fn)->type == OBJ_CLOSURE) {
        ObjClosure *cl = (ObjClosure *)val_as_obj(fn);
        Proto *f = cl->proto;
        /* 顶层直调走解释器（mica_call 家族统一入口）。 */
        ctx->stack[0] = fn;
        for (int i = 2 + argc; i < (int)f->arity + 1; i++) ctx->stack[i] = val_null();
        ctx->stack_top = ctx->stack + (int)f->max_regs;
        CallFrame *f0 = ctx_frame_new(ctx);
        *f0 = (CallFrame){ .closure = cl, .savedpc = f->code, .u = {0}, .base = ctx->stack,
                           .want = 1 };
        f0->argc = (uint16_t)(argc + 1); /* vararg：宿主直调实参数（含 self 槽） */

        const CoroResult r = vm_run(ctx, &result_v, 0);
        if (r == CORO_ERROR) {
            str_set(&ctx->st_error_msg, "%s", ctx->st_error_msg ? ctx->st_error_msg : "");
            ctx->st_error_line = ctx->st_error_line;
            rc = 1;
        }
    } else {
        str_set(&ctx->st_error_msg, "'%s' is not callable", name);
        ctx->st_error_line = 0;
        return 1;
    }
    if (result) *result = wrap(result_v);
    return rc;
}

const char *mica_error_message(const mica_context *ctx) {
    return ctx->st_error_msg ? ctx->st_error_msg : "";
}

int mica_error_line(const mica_context *ctx) {
    return ctx->st_error_line;
}

const char *mica_error_path(const mica_context *ctx) {
    return (ctx->error_path && ctx->error_path[0]) ? ctx->error_path : NULL;
}

const char *mica_runtime_error_path(const mica_runtime *rt) {
    return rt->last_error_path ? rt->last_error_path : "";
}

/* ---------- 值 ---------- */

mica_value mica_null(void) {
    return wrap(val_null());
}

mica_value mica_bool(int b) {
    return wrap(val_bool(b != 0));
}

mica_value mica_int(int64_t i) {
    return wrap(val_int(i));
}

mica_value mica_float(double f) {
    return wrap(val_float(f));
}

mica_value mica_string(mica_runtime *rt, const char *s, size_t len) {
    return wrap(obj_string_val(obj_string_new(&rt->gc, s, len)));
}

int mica_value_type(const mica_value *v) {
    const Value uv = unwrap(*v);
    if (val_is_float(uv)) return MICA_FLOAT; /* number 区：int53 统一 double，无独立 int 类型 */
    switch (val_tag(uv)) {
        case MICA_TNULL: return MICA_NULL;
        case MICA_TFALSE: case MICA_TTRUE: return MICA_BOOL;
        case MICA_TSTR:
        case MICA_TARRAY:
        case MICA_TMAP:
        case MICA_TINSTANCE:
        case MICA_TCLOSURE:
        case MICA_TOBJ:
            switch (val_as_obj(uv)->type) {
                case OBJ_STRING: return MICA_STRING;
                case OBJ_ARRAY: return MICA_ARRAY;
                case OBJ_MAP: return MICA_MAP;
                case OBJ_NATIVE_CLOSURE:
                case OBJ_CLOSURE:
                case OBJ_BUILTIN_METHOD: return MICA_FUNC;
                case OBJ_CLASS: case OBJ_NATIVE_CLASS: return MICA_OBJECT;
                case OBJ_INSTANCE: case OBJ_NATIVE_INSTANCE: return MICA_OBJECT;
                default: return MICA_OBJECT;            }
        default: return MICA_OBJECT;
    }
}

bool mica_is_null(mica_value v) { return val_is_null(unwrap(v)); }
bool mica_is_bool(mica_value v) { return val_is_bool(unwrap(v)); }
bool mica_is_int(mica_value v) { return val_is_num(unwrap(v)); }
bool mica_is_float(mica_value v) { return val_is_float(unwrap(v)); }
bool mica_is_string(mica_value v) {
    return val_is_obj(unwrap(v)) && val_as_obj(unwrap(v))->type == OBJ_STRING;
}

bool mica_as_bool(mica_value v) { return val_as_bool(unwrap(v)); }
int64_t mica_as_int(mica_value v) {
    Value x = unwrap(v);
    /* int 已删：number 一律 double；val_is_num==val_is_float，只判一次 */
    return val_is_num(x) ? (int64_t)val_as_float(x) : 0;
}
double mica_as_float(mica_value v) {
    Value x = unwrap(v);
    return val_is_num(x) ? val_as_float(x) : 0.0;
}

const char *mica_as_string(mica_value v, size_t *len) {
    if (!mica_is_string(v)) {
        if (len) *len = 0;
        return NULL;
    }
    ObjString *s = (ObjString *)val_as_obj(unwrap(v));
    if (len) *len = s->len;
    return s->chars;
}

void mica_print_value(mica_value v) {
    value_print(stdout, unwrap(v));
}
