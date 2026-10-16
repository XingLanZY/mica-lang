/* 反射系统（librefl 模块）：typeof() 返回 Type，配套 Assembly/Module/Method/Field/Property。
   作为原生模块在 mica_runtime_new 时经 mica_register_* API 注入 System.Reflection 命名空间；
   反射对象类型是自定义 GC 类型（LREFL_*，gc_register_vtable 注册）。
   构造/枚举逻辑集中在此：
   - typeof 的 native（builtin.c）调 refl_type_of；
   - vm.c get_member 的反射对象属性与方法分派调这里的构造函数；
   - 模块命名空间枚举沿 rt->globals（ObjTable）按点分名走 ObjMap 链。 */

#include "librefl.h"
#include "state.h"
#include "mica.h"
#include "vm.h"
#include "obj.h"
#include "proto.h"
#include "value.h"
#include "gc.h"
#include "table.h"
#include "str.h"
#include "strlit.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 前向声明（refl_assembly 引用 refl_assembly_named；Assembly.load native；
   librefl_init 注册成员处理器引用 gm_refl_*）。 */
static Value refl_assembly_named(mica_context *ctx, const char *name);
static uint32_t refl_assembly_load(mica_context *ctx, mica_value *args, uint32_t argc,
                                   mica_value *results);
static Value gm_refl_type(mica_context *ctx, CallFrame *fr, Value obj, const String *name);
static Value gm_refl_method(mica_context *ctx, CallFrame *fr, Value obj, const String *name);
static Value gm_refl_field(mica_context *ctx, CallFrame *fr, Value obj, const String *name);
static Value gm_refl_property(mica_context *ctx, CallFrame *fr, Value obj, const String *name);
static Value gm_refl_module(mica_context *ctx, CallFrame *fr, Value obj, const String *name);
static Value gm_refl_assembly(mica_context *ctx, CallFrame *fr, Value obj, const String *name);
static uint32_t native_typeof(mica_context *ctx, mica_value *args, uint32_t argc, mica_value *results);
static uint32_t native_assembly(mica_context *ctx, mica_value *args, uint32_t argc, mica_value *results);

/* 反射对象构造器（定义在后；此处前向声明供 refl_type_* 提前使用）。 */
static ObjReflType *obj_refl_type_new(Gc *gc, ObjString *name, ObjClass *klass, uint8_t prim);
static Value obj_refl_type_val(ObjReflType *t);
static ObjReflMethod *obj_refl_method_new(Gc *gc, ObjString *name, Value func, uint32_t arity,
                                          bool is_static, bool needs_receiver, ObjString *module);
static Value obj_refl_method_val(ObjReflMethod *m);
static ObjReflField *obj_refl_field_new(Gc *gc, ObjString *name);
static Value obj_refl_field_val(ObjReflField *f);
static ObjReflProperty *obj_refl_property_new(Gc *gc, ObjString *name, bool has_getter, bool has_setter,
                                              Value getter, Value setter);
static Value obj_refl_property_val(ObjReflProperty *p);
static ObjReflModule *obj_refl_module_new(Gc *gc, ObjString *name, ObjReflAssembly *assembly);
static Value obj_refl_module_val(ObjReflModule *m);
static ObjReflAssembly *obj_refl_assembly_new(Gc *gc, ObjString *name, mica_runtime *rt);
static Value obj_refl_assembly_val(ObjReflAssembly *a);

/* 标识符 → 方法查找参数（hash, chars, len）。 */
#define NAME_KV(name) (name)->hash, str_data(name), str_len(name)

/* mica_value ↔ Value（mica_value 是 NaN-boxed Value 的透明包装） */
static Value lv(mica_value mv) {
    Value v;
    memcpy(&v, mv.bits, sizeof(v));
    return v;
}

/* RTT_CLASS 的名字由 klass->name 决定（此槽不用）；RTT_ENUM 的名字由枚举值 set_name 决定。 */
static const char *const k_prim_names[RTT_COUNT] = {
    "null", "bool", "number", "string", "array", "map", "function", "error",
    "enum", "coroutine", "WeakRef", "", "Type", "Method", "Field", "Property", "Module",
    "Assembly", "object",
};

/* 内建类型 → Type（name 是刚建的 young 串；unsafe 窗口内 obj_refl_type_new 不触发收集）。 */
static Value refl_type_prim(mica_context *ctx, uint8_t prim) {
    Gc *gc = ctx->gc;
    gc_begin_unsafe(gc);
    ObjString *name = obj_string_new(gc, k_prim_names[prim], strlen(k_prim_names[prim]));
    ObjReflType *t = obj_refl_type_new(gc, name, NULL, prim);
    gc_end_unsafe(gc);
    return obj_refl_type_val(t);
}

/* 枚举值 → Type（name = 枚举名；set_name 是 permanent 常量池串，无需根）。
   结构相等按 name 内容（value.c 的 RTT_ENUM 分支）。 */
static Value refl_type_enum(mica_context *ctx, ObjString *set_name) {
    ObjReflType *t = obj_refl_type_new(ctx->gc, set_name, NULL, RTT_ENUM);
    return obj_refl_type_val(t);
}

/* 用户类 → Type（klass 与 klass->name 均 permanent，无需根）。 */
static Value refl_type_class(mica_context *ctx, ObjClass *klass) {
    ObjReflType *t = obj_refl_type_new(ctx->gc, klass->name, klass, RTT_CLASS);
    return obj_refl_type_val(t);
}

/* 原生类 → Type（name-only；原生类是宿主 C 结构体，无 Mica 字段/方法反射，klass=NULL）。 */
static Value refl_type_native(mica_context *ctx, ObjString *name) {
    ObjReflType *t = obj_refl_type_new(ctx->gc, name, NULL, RTT_CLASS);
    return obj_refl_type_val(t);
}

/* 从 System.Reflection 命名空间查反射类（librefl_init 注册；未注册返回 NULL）。 */
static ObjClass *librefl_class(mica_runtime *rt, const char *name) {
    char flat[512];
    snprintf(flat, sizeof(flat), "System.Reflection.%s", name);
    Value v = lv(mica_runtime_get_global(rt, flat));
    if (val_is_obj(v) && val_as_obj(v)->type == OBJ_CLASS) return (ObjClass *)val_as_obj(v);
    return NULL;
}

/* 反射对象 → 其声明类（System.Reflection 注入的 ObjClass）的 Type；类未注入则回退 prim。 */
static Value refl_type_reflect(mica_context *ctx, const char *cls_name, uint8_t fallback) {
    ObjClass *klass = librefl_class(ctx->rt, cls_name);
    return klass ? refl_type_class(ctx, klass) : refl_type_prim(ctx, fallback);
}

/* ---------- 反射对象构造器（obj.c 移入；自定义类型 LREFL_*） ---------- */

static ObjReflType *obj_refl_type_new(Gc *gc, ObjString *name, ObjClass *klass, uint8_t prim) {
    ObjReflType *t = gc_alloc(gc, sizeof(ObjReflType), LREFL_TYPE);
    t->name = name;
    t->klass = klass;
    t->prim = prim;
    return t;
}

static Value obj_refl_type_val(ObjReflType *t) {
    return val_obj((Obj *)t);
}

static ObjReflMethod *obj_refl_method_new(Gc *gc, ObjString *name, Value func, uint32_t arity,
                                   bool is_static, bool needs_receiver, ObjString *module) {
    ObjReflMethod *m = gc_alloc(gc, sizeof(ObjReflMethod), LREFL_METHOD);
    m->name = name;
    m->func = func;
    m->arity = arity;
    m->is_static = is_static;
    m->needs_receiver = needs_receiver;
    m->module = module;
    return m;
}

static Value obj_refl_method_val(ObjReflMethod *m) {
    return val_obj((Obj *)m);
}

static ObjReflField *obj_refl_field_new(Gc *gc, ObjString *name) {
    ObjReflField *f = gc_alloc(gc, sizeof(ObjReflField), LREFL_FIELD);
    f->name = name;
    return f;
}

static Value obj_refl_field_val(ObjReflField *f) {
    return val_obj((Obj *)f);
}

static ObjReflProperty *obj_refl_property_new(Gc *gc, ObjString *name, bool has_getter, bool has_setter,
                                       Value getter, Value setter) {
    ObjReflProperty *p = gc_alloc(gc, sizeof(ObjReflProperty), LREFL_PROPERTY);
    p->name = name;
    p->has_getter = has_getter;
    p->has_setter = has_setter;
    p->getter = getter;
    p->setter = setter;
    return p;
}

static Value obj_refl_property_val(ObjReflProperty *p) {
    return val_obj((Obj *)p);
}

static ObjReflModule *obj_refl_module_new(Gc *gc, ObjString *name, ObjReflAssembly *assembly) {
    ObjReflModule *m = gc_alloc(gc, sizeof(ObjReflModule), LREFL_MODULE);
    m->name = name;
    m->assembly = assembly;
    return m;
}

static Value obj_refl_module_val(ObjReflModule *m) {
    return val_obj((Obj *)m);
}

static ObjReflAssembly *obj_refl_assembly_new(Gc *gc, ObjString *name, mica_runtime *rt) {
    ObjReflAssembly *a = gc_alloc(gc, sizeof(ObjReflAssembly), LREFL_ASSEMBLY);
    a->name = name;
    a->rt = rt;
    return a;
}

static Value obj_refl_assembly_val(ObjReflAssembly *a) {
    return val_obj((Obj *)a);
}

/* ---------- 反射对象 trace（obj.c 移入） ---------- */

static void refl_type_trace(Obj *obj, GcTraceCtx *ctx) {
    ObjReflType *t = (ObjReflType *)obj;
    gc_trace_obj(ctx, (Obj **)&t->name);
    gc_trace_obj(ctx, (Obj **)&t->klass);
}

static void refl_method_trace(Obj *obj, GcTraceCtx *ctx) {
    ObjReflMethod *m = (ObjReflMethod *)obj;
    gc_trace_obj(ctx, (Obj **)&m->name);
    gc_trace_obj(ctx, (Obj **)&m->module);
    gc_trace_value(ctx, &m->func);
}

static void refl_field_trace(Obj *obj, GcTraceCtx *ctx) {
    gc_trace_obj(ctx, (Obj **)&((ObjReflField *)obj)->name);
}

static void refl_property_trace(Obj *obj, GcTraceCtx *ctx) {
    ObjReflProperty *p = (ObjReflProperty *)obj;
    gc_trace_obj(ctx, (Obj **)&p->name);
    gc_trace_value(ctx, &p->getter);
    gc_trace_value(ctx, &p->setter);
}

static void refl_module_trace(Obj *obj, GcTraceCtx *ctx) {
    ObjReflModule *m = (ObjReflModule *)obj;
    gc_trace_obj(ctx, (Obj **)&m->name);
    gc_trace_obj(ctx, (Obj **)&m->assembly);
}

static void refl_assembly_trace(Obj *obj, GcTraceCtx *ctx) {
    gc_trace_obj(ctx, (Obj **)&((ObjReflAssembly *)obj)->name);
}

/* 注册反射自定义类型的 GC vtable。mica_runtime_new 调用（obj_vtables_init 之后）。 */
/* ---------- 反射自定义类型的语义钩子（注入 value.c，见 obj.h） ----------
   Type 对象按结构相等（prim + klass/name 决定身份）；其余反射类型引用相等。
   放在本模块而非 value.c —— 基础值系统不再依赖反射（依赖倒置）。 */

/* 6 种反射类型共享同一前缀布局（ObjHeader + gclist + name，见 librefl.h），
   故取 name 无需按类型分派。 */
typedef struct { ObjHeader Obj *gclist; ObjString *name; } ReflObjHead;

static bool refl_custom_eq(const Obj *a, const Obj *b) {
    if (a->type == LREFL_TYPE && b->type == LREFL_TYPE) {
        const ObjReflType *rta = (const ObjReflType *)a, *rtb = (const ObjReflType *)b;
        if (rta->prim != rtb->prim) return false;
        if (rta->prim == RTT_CLASS) return rta->klass == rtb->klass;
        if (rta->prim == RTT_ENUM) {
            return rta->name && rtb->name && obj_string_eq(rta->name, rtb->name);
        }
        return true; /* 内建/反射类型：prim 决定身份（name 由 prim 决定） */
    }
    return a == b; /* 其它自定义类型：引用相等 */
}

static uint64_t refl_custom_hash(const Obj *o) {
    if (o->type == LREFL_TYPE) {
        const ObjReflType *rt = (const ObjReflType *)o;
        uint64_t h = hash64((uint64_t)rt->prim);
        if (rt->prim == RTT_CLASS) h ^= hash64((uint64_t)(uintptr_t)rt->klass);
        if (rt->prim == RTT_ENUM && rt->name) h ^= obj_string_hash(rt->name);
        return h;
    }
    return hash64((uint64_t)(uintptr_t)o);
}

/* 类型标签：typeof() 与调试打印用（对应旧 value.c 里的 LREFL_* case）。 */
static const char *refl_custom_kind(const Obj *o) {
    switch (o->type) {
        case LREFL_TYPE:     return "Type";
        case LREFL_METHOD:   return "Method";
        case LREFL_FIELD:    return "Field";
        case LREFL_PROPERTY: return "Property";
        case LREFL_MODULE:   return "Module";
        case LREFL_ASSEMBLY: return "Assembly";
        default:             return "object";
    }
}

static const char *refl_custom_name(const Obj *o) {
    const ObjString *n = ((const ReflObjHead *)o)->name;
    return n ? n->chars : NULL;
}

static void librefl_vtables_init(Gc *gc) {
    gc_register_vtable(gc, LREFL_TYPE, (ObjVtable){refl_type_trace, NULL});
    gc_register_vtable(gc, LREFL_METHOD, (ObjVtable){refl_method_trace, NULL});
    gc_register_vtable(gc, LREFL_FIELD, (ObjVtable){refl_field_trace, NULL});
    gc_register_vtable(gc, LREFL_PROPERTY, (ObjVtable){refl_property_trace, NULL});
    gc_register_vtable(gc, LREFL_MODULE, (ObjVtable){refl_module_trace, NULL});
    gc_register_vtable(gc, LREFL_ASSEMBLY, (ObjVtable){refl_assembly_trace, NULL});
    /* 反射类型均有 gclist、可入 gray 链 —— 由本模块自行向 GC 声明，
       GC 核心不再硬编码 LREFL_* 常量（解除 gc.c → librefl 的反向依赖）。 */
    gc_register_obj_type(gc, LREFL_TYPE, true);
    gc_register_obj_type(gc, LREFL_METHOD, true);
    gc_register_obj_type(gc, LREFL_FIELD, true);
    gc_register_obj_type(gc, LREFL_PROPERTY, true);
    gc_register_obj_type(gc, LREFL_MODULE, true);
    gc_register_obj_type(gc, LREFL_ASSEMBLY, true);
}

/* 注入反射自定义类型的语义钩子（value.c 提供注入点，见 obj.h）。
   未注入时基础值系统退化为引用相等 / 指针哈希 / "<obj>"。 */
static void librefl_hooks_init(void) {
    static const MicaCustomHooks hooks = {
        refl_custom_eq, refl_custom_hash, refl_custom_kind, refl_custom_name,
    };
    value_set_custom_hooks(&hooks);
}

/* 注册 System.Reflection 模块（6 个反射类 + Assembly.load 静态方法）+ 反射对象类型
   vtable + typeof/assembly 内建 + LREFL_* 成员访问处理器。mica_runtime_new 调用（唯一注册接口）。 */
void librefl_init(mica_runtime *rt) {
    librefl_vtables_init(&rt->gc); /* 反射自定义类型（LREFL_*）vtable + grayable 声明 */
    librefl_hooks_init();          /* 注入自定义类型值相等/哈希钩子 */

    mica_register_class(rt, "System.Reflection", "Type");
    mica_register_class(rt, "System.Reflection", "Method");
    mica_register_class(rt, "System.Reflection", "Field");
    mica_register_class(rt, "System.Reflection", "Property");
    mica_register_class(rt, "System.Reflection", "Module");
    mica_register_class(rt, "System.Reflection", "Assembly");
    /* Assembly.load 静态成员：运行时动态加载 .mlib 库（get_member 对 OBJ_CLASS 返回 native）。 */
    Value av = lv(mica_runtime_get_global(rt, "System.Reflection.Assembly"));
    if (val_is_obj(av) && val_as_obj(av)->type == OBJ_CLASS) {
        ObjClass *acl = (ObjClass *)val_as_obj(av);
        ObjNativeClosure *load_nat = obj_native_closure_new_permanent(&rt->gc, refl_assembly_load,
                                                                      "load", 1);
        objtable_set_kv(&rt->gc, acl->static_methods, fnv_hash("load", 4),
                        "load", 4, obj_native_closure_val(load_nat));
    }

    /* 注册 `typeof` / `assembly()` 内建（顶层全局）。 */
    ObjNativeClosure *t = obj_native_closure_new_permanent(&rt->gc, native_typeof, "typeof", 1);
    objtable_set_kv(&rt->gc, rt->globals, fnv_hash("typeof", 6), "typeof", 6,
                    obj_native_closure_val(t));
    ObjNativeClosure *a = obj_native_closure_new_permanent(&rt->gc, native_assembly, "assembly", 0);
    objtable_set_kv(&rt->gc, rt->globals, fnv_hash("assembly", 8), "assembly", 8,
                    obj_native_closure_val(a));

    /* 注册 LREFL_* 类型的成员访问处理器（vm 的 get_member 对自定义类型经它分派）。 */
    vm_register_custom_member(LREFL_TYPE, gm_refl_type);
    vm_register_custom_member(LREFL_METHOD, gm_refl_method);
    vm_register_custom_member(LREFL_FIELD, gm_refl_field);
    vm_register_custom_member(LREFL_PROPERTY, gm_refl_property);
    vm_register_custom_member(LREFL_MODULE, gm_refl_module);
    vm_register_custom_member(LREFL_ASSEMBLY, gm_refl_assembly);
}

/* ---------- typeof() ---------- */

static Value refl_type_of(mica_context *ctx, Value v) {
    if (val_is_float(v)) return refl_type_prim(ctx, RTT_NUMBER);
    switch (val_tag(v)) {
        case MICA_TNULL: return refl_type_prim(ctx, RTT_NULL);
        case MICA_TFALSE: case MICA_TTRUE: return refl_type_prim(ctx, RTT_BOOL);
        case MICA_TSTR:
        case MICA_TARRAY:
        case MICA_TMAP:
        case MICA_TINSTANCE:
        case MICA_TCLOSURE:
        case MICA_TOBJ: {
            Obj *o = val_as_obj(v);
            switch (o->type) {
                case OBJ_STRING: return refl_type_prim(ctx, RTT_STRING);
                case OBJ_ARRAY: return refl_type_prim(ctx, RTT_ARRAY);
                case OBJ_MAP: return refl_type_prim(ctx, RTT_MAP);
                case OBJ_ERROR: return refl_type_prim(ctx, RTT_ERROR);
                case OBJ_ENUM_VAL: return refl_type_enum(ctx, ((ObjEnumVal *)o)->set_name);
                case OBJ_NATIVE_CLOSURE: case OBJ_CLOSURE:
                case OBJ_BUILTIN_METHOD:
                    return refl_type_prim(ctx, RTT_FUNCTION);
                case OBJ_CONTEXT:
                    return obj_context_is_coroutine((const mica_context *)val_as_obj(v))
                               ? refl_type_prim(ctx, RTT_COROUTINE)
                               : refl_type_prim(ctx, RTT_OBJECT);
                case OBJ_WEAKREF: return refl_type_prim(ctx, RTT_WEAKREF);
                case OBJ_CLASS: return refl_type_class(ctx, (ObjClass *)o);
                case OBJ_NATIVE_CLASS: return refl_type_native(ctx, ((ObjNativeClass *)o)->name);
                case OBJ_INSTANCE: return refl_type_class(ctx, ((ObjInstance *)o)->klass);
                case OBJ_NATIVE_INSTANCE:
                    return refl_type_native(ctx, ((ObjNativeInstance *)o)->klass->name);
                case LREFL_TYPE: return refl_type_reflect(ctx, "Type", RTT_TYPE);
                case LREFL_METHOD: return refl_type_reflect(ctx, "Method", RTT_METHOD);
                case LREFL_FIELD: return refl_type_reflect(ctx, "Field", RTT_FIELD);
                case LREFL_PROPERTY: return refl_type_reflect(ctx, "Property", RTT_PROPERTY);
                case LREFL_MODULE: return refl_type_reflect(ctx, "Module", RTT_MODULE);
                case LREFL_ASSEMBLY: return refl_type_reflect(ctx, "Assembly", RTT_ASSEMBLY);
                default: return refl_type_prim(ctx, RTT_OBJECT);
            }
        }
        default: return refl_type_prim(ctx, RTT_OBJECT);
    }
}

static Value refl_type_base(mica_context *ctx, ObjReflType *t) {
    if (t->prim == RTT_CLASS && t->klass && t->klass->superclass) {
        return refl_type_class(ctx, t->klass->superclass);
    }
    return val_null();
}

/* ---------- 类成员枚举（沿继承链；子类优先，父类同名被覆盖的不重复收集） ---------- */

/* 从函数值提取 arity/module（类方法表存 ObjClosure）。
   统一约定（2026-08-15）：proto->arity 恒含 self；反射对外报用户可见参数个数——
   普通函数扣隐藏 self 槽（is_method 扣 0），方法不变（self 是显式参数）。
   协程函数是普通 ObjClosure（proto->is_coroutine），走 OBJ_CLOSURE 路径。 */
static uint32_t func_value_arity(Value fv) {
    if (val_is_obj(fv)) {
        Obj *o = val_as_obj(fv);
        if (o->type == OBJ_CLOSURE) {
            const Proto *p = ((ObjClosure *)o)->proto;
            return (uint32_t)p->arity - (p->is_method ? 0u : 1u);
        }
    }
    return 0;
}

static ObjString *func_value_module(Value fv) {
    if (val_is_obj(fv)) {
        Obj *o = val_as_obj(fv);
        if (o->type == OBJ_CLOSURE) return ((ObjClosure *)o)->proto->module;
    }
    return NULL;
}

static Value make_method(mica_context *ctx, ObjString *name, Value fv, bool is_static,
                         bool needs_receiver) {
    return obj_refl_method_val(obj_refl_method_new(ctx->gc, name, fv, func_value_arity(fv),
                                                   is_static, needs_receiver,
                                                   func_value_module(fv)));
}

/* 已见成员名集合（去重用；键按内容比较）。 */
typedef struct { ObjString **items; size_t n, cap; } SeenNames;

static void seen_add(SeenNames *s, ObjString *name) {
    if (s->n == s->cap) {
        size_t ncap = s->cap ? s->cap * 2 : 8;
        s->items = realloc(s->items, ncap * sizeof(ObjString *));
        assert(s->items != NULL);
        s->cap = ncap;
    }
    s->items[s->n++] = name;
}

static bool seen_has(const SeenNames *s, ObjString *name) {
    for (size_t i = 0; i < s->n; i++)
        if (obj_string_eq(s->items[i], name)) return true;
    return false;
}

static void seen_free(SeenNames *s) { free(s->items); }

typedef struct {
    mica_context *ctx;
    ObjArray *arr;
    bool is_static;
    SeenNames seen;
} MethodCollect;

static int collect_method(const String *key, Value value, void *user) {
    MethodCollect *mc = (MethodCollect *)user;
    /* 调用方 refl_class_methods 在 unsafe 窗口内枚举：本回调的 gc_alloc 不触发收集，
       name/mc->arr 等 C 局部对象安全。 */
    ObjString *name = obj_string_new(mc->ctx->gc, str_data(key), str_len(key));
    if (seen_has(&mc->seen, name)) return 0;
    seen_add(&mc->seen, name);
    Value mv = make_method(mc->ctx, name, value, mc->is_static, !mc->is_static);
    obj_array_push(mc->ctx->gc, mc->arr, mv);
    return 0;
}

static Value refl_class_methods(mica_context *ctx, ObjClass *klass, bool is_static) {
    gc_begin_unsafe(ctx->gc);
    ObjArray *arr = obj_array_new(ctx->gc, 0);
    Value av = obj_array_val(arr);
    MethodCollect mc = {ctx, arr, is_static, {NULL, 0, 0}};
    for (ObjClass *k = klass; k; k = k->superclass) {
        ObjTable *tbl = is_static ? k->static_methods : k->methods;
        objtable_iter(tbl, collect_method, &mc);
    }
    seen_free(&mc.seen);
    gc_end_unsafe(ctx->gc);
    return av;
}

static Value refl_class_fields(mica_context *ctx, ObjClass *klass) {
    gc_begin_unsafe(ctx->gc);
    ObjArray *arr = obj_array_new(ctx->gc, 0);
    Value av = obj_array_val(arr);
    SeenNames seen = {NULL, 0, 0};
    for (ObjClass *k = klass; k; k = k->superclass) {
        for (size_t i = 0; i < k->n_field_names; i++) {
            ObjString *name = k->field_names[i];
            if (seen_has(&seen, name)) continue;
            seen_add(&seen, name);
            ObjReflField *f = obj_refl_field_new(ctx->gc, name);
            obj_array_push(ctx->gc, arr, obj_refl_field_val(f));
        }
    }
    seen_free(&seen);
    gc_end_unsafe(ctx->gc);
    return av;
}

typedef struct {
    mica_context *ctx;
    ObjArray *arr;
    ObjClass *klass;   /* 根类（沿链查 getter/setter 配对用） */
    SeenNames seen;
} PropCollect;

static int collect_getter_prop(const String *key, Value value, void *user) {
    PropCollect *pc = (PropCollect *)user;
    /* 调用方 refl_class_properties 在 unsafe 窗口内枚举（见 collect_method 注释）。 */
    ObjString *name = obj_string_new(pc->ctx->gc, str_data(key), str_len(key));
    if (seen_has(&pc->seen, name)) return 0;
    seen_add(&pc->seen, name);
    Value setter = obj_class_find_setter_kv(pc->klass, key->hash, str_data(key), str_len(key));
    ObjReflProperty *p = obj_refl_property_new(pc->ctx->gc, name, true, !val_is_null(setter), value,
                                               setter);
    obj_array_push(pc->ctx->gc, pc->arr, obj_refl_property_val(p));
    return 0;
}

static int collect_setter_prop(const String *key, Value value, void *user) {
    PropCollect *pc = (PropCollect *)user;
    ObjString *name = obj_string_new(pc->ctx->gc, str_data(key), str_len(key));
    if (seen_has(&pc->seen, name)) return 0; /* 已有 getter（含 setter 配对），跳过 */
    seen_add(&pc->seen, name);
    Value getter = obj_class_find_getter_kv(pc->klass, key->hash, str_data(key), str_len(key));
    ObjReflProperty *p = obj_refl_property_new(pc->ctx->gc, name, !val_is_null(getter), true,
                                               getter, value);
    obj_array_push(pc->ctx->gc, pc->arr, obj_refl_property_val(p));
    return 0;
}

static Value refl_class_properties(mica_context *ctx, ObjClass *klass) {
    gc_begin_unsafe(ctx->gc);
    ObjArray *arr = obj_array_new(ctx->gc, 0);
    Value av = obj_array_val(arr);
    PropCollect pc = {ctx, arr, klass, {NULL, 0, 0}};
    for (ObjClass *k = klass; k; k = k->superclass) objtable_iter(k->getters, collect_getter_prop, &pc);
    for (ObjClass *k = klass; k; k = k->superclass) objtable_iter(k->setters, collect_setter_prop, &pc);
    seen_free(&pc.seen);
    gc_end_unsafe(ctx->gc);
    return av;
}

/* ---------- 命名空间 / 全局解析 ---------- */

/* 按点分名走 globals（ObjTable → 嵌套 ObjMap 链），返回最终值。 */
static bool resolve_global_value(mica_runtime *rt, const char *name, Value *out) {
    char buf[512];
    snprintf(buf, sizeof(buf), "%s", name);
    char *seg = buf;
    Value cur = val_null();
    for (;;) {
        char *dot = strchr(seg, '.');
        if (dot) *dot = '\0';
        const size_t seg_len = strlen(seg);
        const uint32_t seg_hash = fnv_hash(seg, seg_len);
        if (val_is_null(cur)) {
            if (!objtable_get_kv(rt->globals, seg_hash, seg, seg_len, &cur)) return false;
        } else {
            if (!val_is_obj(cur) || val_as_obj(cur)->type != OBJ_MAP) return false;
            if (!obj_map_get_by_string((ObjMap *)val_as_obj(cur), seg, seg_len, seg_hash, &cur))
                return false;
        }
        if (!dot) break;
        seg = dot + 1;
    }
    *out = cur;
    return true;
}

static ObjMap *resolve_module_namespace(mica_runtime *rt, const char *name) {
    Value v;
    if (!resolve_global_value(rt, name, &v)) return NULL;
    if (!val_is_obj(v) || val_as_obj(v)->type != OBJ_MAP) return NULL;
    return (ObjMap *)val_as_obj(v);
}

/* ---------- 模块 / 程序集 ---------- */

static Value refl_assembly(mica_context *ctx) {
    return refl_assembly_named(ctx, ctx->rt->assembly_name);
}

/* 按指定名建 Assembly 对象（Assembly.load 返回加载的库的 Assembly；rt 进程级不 trace）。 */
static Value refl_assembly_named(mica_context *ctx, const char *name) {
    Gc *gc = ctx->gc;
    gc_begin_unsafe(gc);
    ObjString *nm = obj_string_new(gc, name, strlen(name));
    ObjReflAssembly *a = obj_refl_assembly_new(gc, nm, ctx->rt);
    gc_end_unsafe(gc);
    return obj_refl_assembly_val(a);
}

/* Assembly.load(path)：运行时动态加载 .mlib 库（静态成员，System.Reflection.Assembly.load）。
   返回加载的库的 Assembly（名 = 文件基名去扩展名）。 */
static uint32_t refl_assembly_load(mica_context *ctx, mica_value *args, uint32_t argc,
                                   mica_value *results) {
    if (argc < 1 || !val_is_string(val_unwrap(args[0]))) {
        runtime_error(ctx, NULL, "Assembly.load expects a file path");
        return 0;
    }
    ObjString *path = (ObjString *)val_as_obj(val_unwrap(args[0]));
    char *pb = malloc(path->len + 1);
    assert(pb != NULL);
    memcpy(pb, path->chars, path->len);
    pb[path->len] = '\0';
    const int rc = mica_runtime_load_library(ctx->rt, pb);
    free(pb);
    if (rc != 0) {
        runtime_error(ctx, NULL, "Assembly.load failed: %s", mica_runtime_error(ctx->rt));
        return 0;
    }
    /* 文件基名（去目录与扩展名） */
    const char *base = path->chars;
    for (size_t i = 0; i < path->len; i++) {
        if (path->chars[i] == '/' || path->chars[i] == '\\') base = &path->chars[i + 1];
    }
    size_t blen = path->len - (size_t)(base - path->chars);
    for (size_t i = 0; i < blen; i++) {
        if (base[i] == '.') { blen = i; break; }
    }
    char namebuf[512];
    if (blen >= sizeof(namebuf)) blen = sizeof(namebuf) - 1;
    memcpy(namebuf, base, blen);
    namebuf[blen] = '\0';
    results[0] = val_wrap(refl_assembly_named(ctx, namebuf));
    return 1;
}

static Value refl_module(mica_context *ctx, const char *module_name) {
    if (!resolve_module_namespace(ctx->rt, module_name)) return val_null();
    Value asmval = refl_assembly(ctx);
    gc_begin_unsafe(ctx->gc);
    ObjString *mname = obj_string_new(ctx->gc, module_name, strlen(module_name));
    ObjReflModule *m = obj_refl_module_new(ctx->gc, mname, (ObjReflAssembly *)val_as_obj(asmval));
    gc_end_unsafe(ctx->gc);
    return obj_refl_module_val(m);
}

static Value refl_module_get_type(mica_context *ctx, const char *module_name, ObjString *name) {
    ObjMap *ns = resolve_module_namespace(ctx->rt, module_name);
    if (!ns) return val_null();
    Value v;
    if (obj_map_get(ns, obj_string_val(name), &v) && val_is_obj(v) &&
        val_as_obj(v)->type == OBJ_CLASS) {
        return refl_type_class(ctx, (ObjClass *)val_as_obj(v));
    }
    return val_null();
}

typedef struct {
    mica_context *ctx;
    ObjArray *arr;
} ReflCollect;

static int collect_class_type(const String *key, Value value, void *user) {
    if (memchr(str_data(key), '.', str_len(key))) return 0; /* 跳过模块扁平键（module.Class，命名空间覆盖） */
    ReflCollect *rc = (ReflCollect *)user;
    if (val_is_obj(value) && val_as_obj(value)->type == OBJ_CLASS) {
        Value tv = refl_type_class(rc->ctx, (ObjClass *)val_as_obj(value));
        obj_array_push(rc->ctx->gc, rc->arr, tv);
    }
    return 0;
}

static void collect_class_type_map(ObjString *key, Value value, void *user) {
    (void)key;
    ReflCollect *rc = (ReflCollect *)user;
    if (val_is_obj(value) && val_as_obj(value)->type == OBJ_CLASS) {
        Value tv = refl_type_class(rc->ctx, (ObjClass *)val_as_obj(value));
        obj_array_push(rc->ctx->gc, rc->arr, tv);
    }
}

static void collect_module_func_map(ObjString *key, Value value, void *user) {
    ReflCollect *rc = (ReflCollect *)user;
    if (!val_is_obj(value) || val_as_obj(value)->type != OBJ_CLOSURE) return;
    Value mv = make_method(rc->ctx, key, value, false, false);
    obj_array_push(rc->ctx->gc, rc->arr, mv);
}

static Value refl_module_types(mica_context *ctx, const char *module_name) {
    ObjMap *ns = resolve_module_namespace(ctx->rt, module_name);
    gc_begin_unsafe(ctx->gc);
    ObjArray *arr = obj_array_new(ctx->gc, 0);
    Value av = obj_array_val(arr);
    ReflCollect rc = {ctx, arr};
    if (ns) obj_map_iter(ns, collect_class_type_map, &rc);
    gc_end_unsafe(ctx->gc);
    return av;
}

static Value refl_module_functions(mica_context *ctx, const char *module_name) {
    ObjMap *ns = resolve_module_namespace(ctx->rt, module_name);
    gc_begin_unsafe(ctx->gc);
    ObjArray *arr = obj_array_new(ctx->gc, 0);
    Value av = obj_array_val(arr);
    ReflCollect rc = {ctx, arr};
    if (ns) obj_map_iter(ns, collect_module_func_map, &rc);
    gc_end_unsafe(ctx->gc);
    return av;
}

static Value refl_assembly_types(mica_context *ctx) {
    mica_runtime *rt = ctx->rt;
    gc_begin_unsafe(ctx->gc);
    ObjArray *arr = obj_array_new(ctx->gc, 0);
    Value av = obj_array_val(arr);
    ReflCollect rc = {ctx, arr};
    objtable_iter(rt->globals, collect_class_type, &rc);
    for (size_t i = 0; i < rt->n_loaded; i++) {
        ObjMap *ns = resolve_module_namespace(rt, rt->loaded_modules[i]);
        if (ns) obj_map_iter(ns, collect_class_type_map, &rc);
    }
    gc_end_unsafe(ctx->gc);
    return av;
}

static Value refl_assembly_get_type(mica_context *ctx, ObjString *name) {
    Value v;
    if (resolve_global_value(ctx->rt, name->chars, &v) && val_is_obj(v) &&
        val_as_obj(v)->type == OBJ_CLASS) {
        return refl_type_class(ctx, (ObjClass *)val_as_obj(v));
    }
    return val_null();
}

static Value refl_assembly_modules(mica_context *ctx) {
    mica_runtime *rt = ctx->rt;
    Value asmval = refl_assembly(ctx);
    gc_begin_unsafe(ctx->gc);
    ObjArray *arr = obj_array_new(ctx->gc, 0);
    Value av = obj_array_val(arr);
    for (size_t i = 0; i < rt->n_loaded; i++) {
        if (!resolve_module_namespace(rt, rt->loaded_modules[i])) continue;
        ObjString *mname = obj_string_new(ctx->gc, rt->loaded_modules[i],
                                          strlen(rt->loaded_modules[i]));
        ObjReflModule *m = obj_refl_module_new(ctx->gc, mname, (ObjReflAssembly *)val_as_obj(asmval));
        obj_array_push(ctx->gc, arr, obj_refl_module_val(m));
    }
    gc_end_unsafe(ctx->gc);
    return av;
}

/* 自定义类型值相等 / 哈希（value_eq_custom/value_hash_custom）在 value.c 实现（core 钩子）。 */

/* ============================================================
   反射对象方法（bm_refl_*）+ typeof/assembly 内建 + LREFL_* 成员访问（gm_refl_*）。
   原在 builtin.c / vm.c，随 librefl 模块化收拢到此（lib 模块 = 一个标准库）。
   ============================================================ */

/* ---------- 反射对象方法 dispatch（builtin.c 移入） ---------- */

typedef struct { const char *name; uint8_t len; uint32_t hash; mica_native_fn fn; } ReflMethodEntry;

static uint32_t bm_refl_type_methods(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    (void)argc;
    ObjReflType *t = (ObjReflType *)val_as_obj(args[0]);
    if (t->prim != RTT_CLASS || !t->klass) {
        res[0] = obj_array_val(obj_array_new(ctx->gc, 0));
        return 1;
    }
    res[0] = refl_class_methods(ctx, t->klass, false);
    return 1;
}

static uint32_t bm_refl_type_static_methods(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    (void)argc;
    ObjReflType *t = (ObjReflType *)val_as_obj(args[0]);
    if (t->prim != RTT_CLASS || !t->klass) {
        res[0] = obj_array_val(obj_array_new(ctx->gc, 0));
        return 1;
    }
    res[0] = refl_class_methods(ctx, t->klass, true);
    return 1;
}

static uint32_t bm_refl_type_fields(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    (void)argc;
    ObjReflType *t = (ObjReflType *)val_as_obj(args[0]);
    if (t->prim != RTT_CLASS || !t->klass) {
        res[0] = obj_array_val(obj_array_new(ctx->gc, 0));
        return 1;
    }
    res[0] = refl_class_fields(ctx, t->klass);
    return 1;
}

static uint32_t bm_refl_type_properties(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    (void)argc;
    ObjReflType *t = (ObjReflType *)val_as_obj(args[0]);
    if (t->prim != RTT_CLASS || !t->klass) {
        res[0] = obj_array_val(obj_array_new(ctx->gc, 0));
        return 1;
    }
    res[0] = refl_class_properties(ctx, t->klass);
    return 1;
}

static uint32_t bm_refl_field_get(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    ObjReflField *f = (ObjReflField *)val_as_obj(args[0]);
    if (argc < 2 || !val_is_obj(args[1]) || val_as_obj(args[1])->type != OBJ_INSTANCE) {
        runtime_error(ctx, NULL, "field.get expects an instance");
        return 0;
    }
    ObjInstance *inst = (ObjInstance *)val_as_obj(args[1]);
    const int idx = obj_class_field_index_kv(inst->klass, obj_string_hash(f->name), f->name->chars, f->name->len);
    res[0] = idx >= 0 ? inst->fields[idx] : val_null();
    return 1;
}

static uint32_t bm_refl_field_set(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    (void)res;
    ObjReflField *f = (ObjReflField *)val_as_obj(args[0]);
    if (argc < 3 || !val_is_obj(args[1]) || val_as_obj(args[1])->type != OBJ_INSTANCE) {
        runtime_error(ctx, NULL, "field.set expects an instance and a value");
        return 0;
    }
    ObjInstance *inst = (ObjInstance *)val_as_obj(args[1]);
    String name = str_new(f->name->chars, f->name->len);
    const bool ok = obj_instance_set_field(ctx->gc, inst, &name, args[2]);
    str_free(&name);
    if (!ok) { runtime_error(ctx, NULL, "instance has no field '%s'", f->name->chars); return 0; }
    return 0;
}

static uint32_t bm_refl_module_types(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    (void)argc;
    ObjReflModule *m = (ObjReflModule *)val_as_obj(args[0]);
    res[0] = refl_module_types(ctx, m->name->chars);
    return 1;
}

static uint32_t bm_refl_module_functions(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    (void)argc;
    ObjReflModule *m = (ObjReflModule *)val_as_obj(args[0]);
    res[0] = refl_module_functions(ctx, m->name->chars);
    return 1;
}

static uint32_t bm_refl_module_get_type(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 2 || !val_is_string(args[1])) {
        res[0] = val_null();
        return 1;
    }
    ObjReflModule *m = (ObjReflModule *)val_as_obj(args[0]);
    res[0] = refl_module_get_type(ctx, m->name->chars, (ObjString *)val_as_obj(args[1]));
    return 1;
}

static uint32_t bm_refl_assembly_types(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    (void)args;
    (void)argc;
    res[0] = refl_assembly_types(ctx);
    return 1;
}

static uint32_t bm_refl_assembly_get_type(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 2 || !val_is_string(args[1])) {
        res[0] = val_null();
        return 1;
    }
    res[0] = refl_assembly_get_type(ctx, (ObjString *)val_as_obj(args[1]));
    return 1;
}

static uint32_t bm_refl_assembly_modules(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    (void)args;
    (void)argc;
    res[0] = refl_assembly_modules(ctx);
    return 1;
}

static uint32_t bm_refl_assembly_module(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 2 || !val_is_string(args[1])) {
        res[0] = val_null();
        return 1;
    }
    res[0] = refl_module(ctx, ((ObjString *)val_as_obj(args[1]))->chars);
    return 1;
}

/* ---------- Method.invoke / Property.get / Property.set（反射调用） ---------- */

static uint32_t bm_refl_method_invoke(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    ObjReflMethod *m = (ObjReflMethod *)val_as_obj(args[0]);
    if (ctx->frame_count <= 0) {
        runtime_error(ctx, NULL, "method.invoke: no active frame");
        return 0;
    }
    CallFrame *fr = ctx_frame(ctx);
    if (!val_is_obj(m->func) || val_as_obj(m->func)->type != OBJ_CLOSURE) {
        runtime_error(ctx, fr, "cannot invoke method '%s' directly (coroutine method; use .create())",
                      m->name->chars);
        return 0;
    }
    if (m->needs_receiver && argc < 2) {
        runtime_error(ctx, fr, "instance method '%s' requires a receiver in invoke()",
                      m->name->chars);
        return 0;
    }
    /* args[0]=Method；实例方法 args[1]=receiver、静态方法 args[1..] 即实参。
       统一约定（2026-08-15）：self 显式传、实参从 slot2。 */
    const Value rself = m->needs_receiver ? args[1] : val_null();
    const Value* rargs = &args[1 + (m->needs_receiver ? 1 : 0)];
    const int rn = (int)argc - 1 - (m->needs_receiver ? 1 : 0);
    Value out;
    if (!vm_call_value(ctx, fr, m->func, rself, rargs, rn, &out)) return 0;
    res[0] = out;
    return 1;
}

static uint32_t bm_refl_property_get(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    ObjReflProperty *p = (ObjReflProperty *)val_as_obj(args[0]);
    if (ctx->frame_count <= 0) {
        runtime_error(ctx, NULL, "property.get: no active frame");
        return 0;
    }
    CallFrame *fr = ctx_frame(ctx);
    if (!p->has_getter || val_is_null(p->getter)) {
        runtime_error(ctx, fr, "property '%s' has no getter", p->name->chars);
        return 0;
    }
    if (argc < 2) {
        runtime_error(ctx, fr, "property.get requires an instance");
        return 0;
    }
    Value out;
    if (!vm_call_value(ctx, fr, p->getter, args[1], NULL, 0, &out)) return 0;
    res[0] = out;
    return 1;
}

static uint32_t bm_refl_property_set(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    (void)res;
    ObjReflProperty *p = (ObjReflProperty *)val_as_obj(args[0]);
    if (ctx->frame_count <= 0) {
        runtime_error(ctx, NULL, "property.set: no active frame");
        return 0;
    }
    CallFrame *fr = ctx_frame(ctx);
    if (!p->has_setter || val_is_null(p->setter)) {
        runtime_error(ctx, fr, "property '%s' is read-only", p->name->chars);
        return 0;
    }
    if (argc < 3) {
        runtime_error(ctx, fr, "property.set requires an instance and a value");
        return 0;
    }
    Value out;
    if (!vm_call_value(ctx, fr, p->setter, args[1], &args[2], 1, &out)) return 0;
    return 0;
}

/* Type.get_class()：返回类 Type 对应的类对象（可 `Cls(args)` 构造）；非类 Type 返回 null。 */
static uint32_t bm_refl_type_get_class(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    (void)ctx;
    (void)argc;
    ObjReflType *t = (ObjReflType *)val_as_obj(args[0]);
    res[0] = (t->prim == RTT_CLASS && t->klass) ? obj_class_val(t->klass) : val_null();
    return 1;
}

static const ReflMethodEntry k_refl_type_methods[] = {
    {"methods", 7, H_METHODS, bm_refl_type_methods},
    {"static_methods", 14, H_STATIC_METHODS, bm_refl_type_static_methods},
    {"fields", 6, H_FIELDS, bm_refl_type_fields},
    {"properties", 10, H_PROPERTIES, bm_refl_type_properties},
    {"get_class", 9, H_GET_CLASS, bm_refl_type_get_class},
};
static const ReflMethodEntry k_refl_field_methods[] = {
    {"get", 3, H_GET, bm_refl_field_get}, {"set", 3, H_SET, bm_refl_field_set},
};
static const ReflMethodEntry k_refl_method_methods[] = {
    {"invoke", 6, H_INVOKE, bm_refl_method_invoke},
};
static const ReflMethodEntry k_refl_property_methods[] = {
    {"get", 3, H_GET, bm_refl_property_get}, {"set", 3, H_SET, bm_refl_property_set},
};
static const ReflMethodEntry k_refl_module_methods[] = {
    {"types", 5, H_TYPES, bm_refl_module_types}, {"get_type", 8, H_GET_TYPE, bm_refl_module_get_type},
    {"functions", 9, H_FUNCTIONS, bm_refl_module_functions},
};
static const ReflMethodEntry k_refl_assembly_methods[] = {
    {"types", 5, H_TYPES, bm_refl_assembly_types}, {"get_type", 8, H_GET_TYPE, bm_refl_assembly_get_type},
    {"modules", 7, H_MODULES, bm_refl_assembly_modules}, {"module", 6, H_MODULE, bm_refl_assembly_module},
    {"get_module", 10, H_GET_MODULE, bm_refl_assembly_module}, /* 旧名别名（module 曾是关键字） */
};

/* LREFL_* 类型的方法查找（vm 的 get_member 对反射对象经 gm_refl_* 调这里）。 */
static mica_native_fn find_refl_method(uint32_t type, uint32_t hash, const char *chars, size_t len) {
    const ReflMethodEntry *tbl;
    size_t n;
    switch (type) {
        case LREFL_TYPE: tbl = k_refl_type_methods; n = sizeof(k_refl_type_methods) / sizeof(k_refl_type_methods[0]); break;
        case LREFL_FIELD: tbl = k_refl_field_methods; n = sizeof(k_refl_field_methods) / sizeof(k_refl_field_methods[0]); break;
        case LREFL_METHOD: tbl = k_refl_method_methods; n = sizeof(k_refl_method_methods) / sizeof(k_refl_method_methods[0]); break;
        case LREFL_PROPERTY: tbl = k_refl_property_methods; n = sizeof(k_refl_property_methods) / sizeof(k_refl_property_methods[0]); break;
        case LREFL_MODULE: tbl = k_refl_module_methods; n = sizeof(k_refl_module_methods) / sizeof(k_refl_module_methods[0]); break;
        case LREFL_ASSEMBLY: tbl = k_refl_assembly_methods; n = sizeof(k_refl_assembly_methods) / sizeof(k_refl_assembly_methods[0]); break;
        default: return NULL;
    }
    for (size_t i = 0; i < n; i++) {
        const ReflMethodEntry *e = &tbl[i];
        if (e->len == len && e->hash == hash && memcmp(e->name, chars, len) == 0) return e->fn;
    }
    return NULL;
}

/* ---------- typeof / assembly 内建（builtin.c 移入） ---------- */

static uint32_t native_typeof(mica_context *ctx, mica_value *args, uint32_t argc,
                              mica_value *results) {
    results[0] = val_wrap(refl_type_of(ctx, (argc > 0) ? val_unwrap(args[0]) : val_null()));
    return 1;
}

/* 全局 `assembly()`：当前 runtime 的反射 Assembly（包装 globals + 已加载模块）。 */
static uint32_t native_assembly(mica_context *ctx, mica_value *args, uint32_t argc,
                                mica_value *results) {
    (void)args;
    (void)argc;
    results[0] = val_wrap(refl_assembly(ctx));
    return 1;
}

/* ---------- LREFL_* 成员访问处理器（vm.c 移入） ---------- */

/* Type 的 is_* 谓词哈希表：name->hash 低 6 位（&63）对这 12 个 is_* 名是完美哈希。
   命中且内容相符 → 返回与 t->prim 比较的目标 ReflPrim；is_primitive 用 RTT_COUNT 特判。 */
#define IS_PRIM_BUCKETS 64

typedef struct {
    uint8_t len;
    uint32_t hash;
    const char *lit;
    ReflPrim prim; /* 与 t->prim 比较；RTT_COUNT = is_primitive 特判 */
} IsPrimEntry;

static const IsPrimEntry k_is_prim_tab[IS_PRIM_BUCKETS] = {
    [61] = { sizeof("is_number") - 1, H_IS_NUMBER, "is_number", RTT_NUMBER },
    [11] = { sizeof("is_weakref") - 1, H_IS_WEAKREF, "is_weakref", RTT_WEAKREF },
    [32] = { sizeof("is_function") - 1, H_IS_FUNCTION, "is_function", RTT_FUNCTION },
    [38] = { sizeof("is_null") - 1, H_IS_NULL, "is_null", RTT_NULL },
    [0] = { sizeof("is_bool") - 1, H_IS_BOOL, "is_bool", RTT_BOOL },
    [36] = { sizeof("is_coroutine") - 1, H_IS_COROUTINE, "is_coroutine", RTT_COROUTINE },
    [6] = { sizeof("is_array") - 1, H_IS_ARRAY, "is_array", RTT_ARRAY },
    [8] = { sizeof("is_class") - 1, H_IS_CLASS, "is_class", RTT_CLASS },
    [42] = { sizeof("is_error") - 1, H_IS_ERROR, "is_error", RTT_ERROR },
    [43] = { sizeof("is_string") - 1, H_IS_STRING, "is_string", RTT_STRING },
    [31] = { sizeof("is_map") - 1, H_IS_MAP, "is_map", RTT_MAP },
    [29] = { sizeof("is_primitive") - 1, H_IS_PRIMITIVE, "is_primitive", RTT_COUNT },
};

static bool refl_type_is_pred(const ObjReflType *t, const String *name, bool *out) {
    const IsPrimEntry *e = &k_is_prim_tab[name->hash & (IS_PRIM_BUCKETS - 1)];
    if (e->lit == NULL || e->len != str_len(name) || e->hash != name->hash ||
        memcmp(e->lit, str_data(name), e->len) != 0) {
        return false; /* 非 is_* 名或哈希碰撞 */
    }
    if (e->prim == RTT_COUNT) {
        *out = (t->prim < RTT_CLASS || t->prim == RTT_OBJECT); /* is_primitive 特判 */
    } else {
        *out = (t->prim == e->prim);
    }
    return true;
}

static Value gm_refl_type(mica_context *ctx, CallFrame *fr, Value obj, const String *name) {
    ObjReflType *t = (ObjReflType *)val_as_obj(obj);
    if (STR_EQ(name, H_NAME, "name")) return obj_string_val(t->name);
    if (STR_EQ(name, H_FULL_NAME, "full_name")) {
        if (t->prim == RTT_CLASS && t->klass && t->klass->module) {
            const ObjString *mod = t->klass->module;
            const ObjString *cn = t->klass->name;
            const size_t len = mod->len + 1 + cn->len;
            char *buf = malloc(len + 1);
            assert(buf != NULL);
            memcpy(buf, mod->chars, mod->len);
            buf[mod->len] = '.';
            memcpy(buf + mod->len + 1, cn->chars, cn->len);
            buf[len] = '\0';
            const Value v = obj_string_val(obj_string_new(ctx->gc, buf, len));
            free(buf);
            return v;
        }
        return obj_string_val(t->name);
    }
    if (STR_EQ(name, H_BASE_TYPE, "base_type")) return refl_type_base(ctx, t);
    if (STR_EQ(name, H_ASSEMBLY, "assembly")) return refl_assembly(ctx); {
        bool isv;
        if (refl_type_is_pred(t, name, &isv)) return val_bool(isv);
    } {
        mica_native_fn fn = find_refl_method(LREFL_TYPE, NAME_KV(name));
        if (fn) return obj_builtin_method_val(obj_builtin_method_new(ctx->gc, fn));
    }
    runtime_error(ctx, fr, "Type has no member '%s'", str_data(name));
    return val_null();
}

static Value gm_refl_method(mica_context *ctx, CallFrame *fr, Value obj, const String *name) {
    ObjReflMethod *m = (ObjReflMethod *)val_as_obj(obj);
    if (STR_EQ(name, H_NAME, "name")) return obj_string_val(m->name);
    if (STR_EQ(name, H_ARITY, "arity")) return val_int((int64_t)m->arity);
    if (STR_EQ(name, H_IS_STATIC, "is_static")) return val_bool(m->is_static);
    if (STR_EQ(name, H_MODULE, "module"))
        return m->module ? obj_string_val(m->module) : val_null(); {
        mica_native_fn fn = find_refl_method(LREFL_METHOD, NAME_KV(name));
        if (fn) return obj_builtin_method_val(obj_builtin_method_new(ctx->gc, fn));
    }
    runtime_error(ctx, fr, "Method has no member '%s'", str_data(name));
    return val_null();
}

static Value gm_refl_field(mica_context *ctx, CallFrame *fr, Value obj, const String *name) {
    ObjReflField *f = (ObjReflField *)val_as_obj(obj);
    if (STR_EQ(name, H_NAME, "name")) return obj_string_val(f->name);
    if (STR_EQ(name, H_IS_STATIC, "is_static")) return val_false(); {
        mica_native_fn fn = find_refl_method(LREFL_FIELD, NAME_KV(name));
        if (fn) return obj_builtin_method_val(obj_builtin_method_new(ctx->gc, fn));
    }
    runtime_error(ctx, fr, "Field has no member '%s'", str_data(name));
    return val_null();
}

static Value gm_refl_property(mica_context *ctx, CallFrame *fr, Value obj, const String *name) {
    ObjReflProperty *p = (ObjReflProperty *)val_as_obj(obj);
    if (STR_EQ(name, H_NAME, "name")) return obj_string_val(p->name);
    if (STR_EQ(name, H_CAN_READ, "can_read")) return val_bool(p->has_getter);
    if (STR_EQ(name, H_CAN_WRITE, "can_write")) return val_bool(p->has_setter); {
        mica_native_fn fn = find_refl_method(LREFL_PROPERTY, NAME_KV(name));
        if (fn) return obj_builtin_method_val(obj_builtin_method_new(ctx->gc, fn));
    }
    runtime_error(ctx, fr, "Property has no member '%s'", str_data(name));
    return val_null();
}

static Value gm_refl_module(mica_context *ctx, CallFrame *fr, Value obj, const String *name) {
    ObjReflModule *m = (ObjReflModule *)val_as_obj(obj);
    if (STR_EQ(name, H_NAME, "name")) return obj_string_val(m->name);
    if (STR_EQ(name, H_ASSEMBLY, "assembly")) return obj_refl_assembly_val(m->assembly); {
        mica_native_fn fn = find_refl_method(LREFL_MODULE, NAME_KV(name));
        if (fn) return obj_builtin_method_val(obj_builtin_method_new(ctx->gc, fn));
    }
    runtime_error(ctx, fr, "Module has no member '%s'", str_data(name));
    return val_null();
}

static Value gm_refl_assembly(mica_context *ctx, CallFrame *fr, Value obj, const String *name) {
    ObjReflAssembly *a = (ObjReflAssembly *)val_as_obj(obj);
    if (STR_EQ(name, H_NAME, "name")) return obj_string_val(a->name); {
        mica_native_fn fn = find_refl_method(LREFL_ASSEMBLY, NAME_KV(name));
        if (fn) return obj_builtin_method_val(obj_builtin_method_new(ctx->gc, fn));
    }
    runtime_error(ctx, fr, "Assembly has no member '%s'", str_data(name));
    return val_null();
}
