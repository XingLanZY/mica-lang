/* 程序集序列化/反序列化。
 *
 * 文件格式（小端）：
 *   [0..7]   魔数 "MICAASM"
 *   [8..11]  version u32 = 11（arity 恒含 self）
 *   [12..15] n_modules u32
 *   [..]     模块 i：{ name:str, exports:u32, exports[]:str, init_id:u32 }
 *   [..]     entry_id:u32
 *   [..]     n_objects:u32
 *   [..]     对象类型表：n_objects × u8（0=ObjClosure，1=ObjClass）
 *   [..]     对象数据：n_objects 个定义，按 id 顺序
 *
 * 字符串 = u32 len + bytes。NONE（如无父类/无 defaults）写 0xFFFFFFFF。
 * 对象图先整体编号（collect 阶段深度优先收集全部 ObjClosure/ObjClass，去重），
 * 再按 id 写定义；常量里的对象引用写 id，天然处理环。
 */

#include "assembly.h"

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "proto.h"
#include "table.h"

/* ---------- Writer / Reader 原语 ---------- */

typedef struct { uint8_t *buf; size_t len, cap; } Writer;

static void w_ensure(Writer *w, size_t n) {
    if (w->len + n > w->cap) {
        size_t nc = w->cap ? w->cap * 2 : 256;
        while (nc < w->len + n) nc *= 2;
        w->buf = realloc(w->buf, nc);
        assert(w->buf != NULL);
        w->cap = nc;
    }
}
static void w_bytes(Writer *w, const void *p, size_t n) {
    w_ensure(w, n);
    memcpy(w->buf + w->len, p, n);
    w->len += n;
}
static void w_u8(Writer *w, uint8_t v) { w_bytes(w, &v, 1); }
static void w_u32(Writer *w, uint32_t v) { w_bytes(w, &v, 4); }
static void w_i32(Writer *w, int32_t v) { w_bytes(w, &v, 4); }
static void w_i64(Writer *w, int64_t v) { w_bytes(w, &v, 8); }
static void w_f64(Writer *w, double v) { w_bytes(w, &v, 8); }
static void w_str(Writer *w, const char *s, size_t n) {
    w_u32(w, (uint32_t)n);
    if (n) w_bytes(w, s, n);
}

/* 标识符名直写：proto_add_ident 已把 "System.*" 压缩为 "@X"（运行时规范前缀），
   .mlib 里的标识符池就是运行时的 @ 形式，无需再变换。 */

typedef struct { const uint8_t *p; const uint8_t *end; } Reader;

/* 有界对象访问：id 越界返回 NULL（损坏/截断的 .mlib 应报错而非越界读）。 */
#define OBJ_AT(objs, n_objs, id) ((id) < (n_objs) ? (objs)[id] : NULL)

static bool r_need(Reader *r, size_t n) { return r->p + n <= r->end; }
static uint8_t r_u8(Reader *r) { return r_need(r, 1) ? *r->p++ : 0; }
static uint32_t r_u32(Reader *r) {
    uint32_t v = 0;
    if (r_need(r, 4)) { memcpy(&v, r->p, 4); r->p += 4; }
    return v;
}
static int32_t r_i32(Reader *r) {
    int32_t v = 0;
    if (r_need(r, 4)) { memcpy(&v, r->p, 4); r->p += 4; }
    return v;
}
static int64_t r_i64(Reader *r) {
    int64_t v = 0;
    if (r_need(r, 8)) { memcpy(&v, r->p, 8); r->p += 8; }
    return v;
}
static double r_f64(Reader *r) {
    double v = 0;
    if (r_need(r, 8)) { memcpy(&v, r->p, 8); r->p += 8; }
    return v;
}
/* 读字符串：返回 malloc 的 NUL 结尾副本（调用方 free），失败返回 NULL。 */
static char *r_str(Reader *r, size_t *out_len) {
    const uint32_t n = r_u32(r);
    if (n > (uint32_t)(r->end - r->p)) { r->p = r->end; if (out_len) *out_len = 0; return NULL; }
    char *s = malloc((size_t)n + 1);
    if (!s) { r->p = r->end; if (out_len) *out_len = 0; return NULL; }
    memcpy(s, r->p, n);
    s[n] = '\0';
    r->p += n;
    if (out_len) *out_len = n;
    return s;
}


/* ---------- collect：对象图编号 ---------- */

typedef struct {
    Gc *gc;
    ObjClosure **funcs; size_t nf, cf;
    ObjClass **classes; size_t nc, cc;
} ObjGraph;

static void graph_collect_func(ObjGraph *g, ObjClosure *f);
static void graph_collect_class(ObjGraph *g, ObjClass *c);

static void graph_collect_value(ObjGraph *g, Value v) {
    if (!val_is_obj(v)) return;
    Obj *o = val_as_obj(v);
    if (o->type == OBJ_CLOSURE) graph_collect_func(g, (ObjClosure *)o);
    else if (o->type == OBJ_CLASS) graph_collect_class(g, (ObjClass *)o);
}

static void graph_collect_proto(ObjGraph *g, Proto *p) {
    for (size_t i = 0; i < p->n_constants; i++) graph_collect_value(g, p->constants[i]);
    for (size_t i = 0; i < p->n_num_consts; i++) graph_collect_value(g, p->num_consts[i]);
    for (size_t i = 0; i < p->n_str_consts; i++) graph_collect_value(g, p->str_consts[i]);
}

static void graph_collect_func(ObjGraph *g, ObjClosure *f) {
    if (!f) return;
    for (size_t i = 0; i < g->nf; i++) if (g->funcs[i] == f) return;
    if (g->nf == g->cf) {
        g->cf = g->cf ? g->cf * 2 : 16;
        g->funcs = realloc(g->funcs, g->cf * sizeof(*g->funcs));
        assert(g->funcs != NULL);
    }
    g->funcs[g->nf++] = f;
    graph_collect_proto(g, f->proto);
}

static int graph_table_cb(const String *key, Value v, void *u) {
    (void)key;
    Obj *o = val_as_obj(v);
    if (o->type == OBJ_CLOSURE) graph_collect_func((ObjGraph *)u, (ObjClosure *)o);
    return 0;
}

static void graph_collect_class(ObjGraph *g, ObjClass *c) {
    if (!c) return;
    for (size_t i = 0; i < g->nc; i++) if (g->classes[i] == c) return;
    if (g->nc == g->cc) {
        g->cc = g->cc ? g->cc * 2 : 8;
        g->classes = realloc(g->classes, g->cc * sizeof(*g->classes));
        assert(g->classes != NULL);
    }
    g->classes[g->nc++] = c;
    if (c->superclass) graph_collect_class(g, c->superclass);
    if (c->defaults) graph_collect_func(g, c->defaults);
    objtable_iter(c->methods, graph_table_cb, g);
    objtable_iter(c->static_methods, graph_table_cb, g);
    objtable_iter(c->getters, graph_table_cb, g);
    objtable_iter(c->setters, graph_table_cb, g);
}

static size_t graph_func_id(ObjGraph *g, ObjClosure *f) {
    for (size_t i = 0; i < g->nf; i++) if (g->funcs[i] == f) return i;
    return 0;
}
static size_t graph_class_id(ObjGraph *g, ObjClass *c) {
    for (size_t i = 0; i < g->nc; i++) if (g->classes[i] == c) return i;
    return 0;
}
/* 统一对象 id：func 0..nf-1，class nf..nf+nc-1（ObjClosure 映射到其 func）。 */
static uint32_t graph_obj_id(ObjGraph *g, Value v) {
    if (!val_is_obj(v)) return 0;
    Obj *o = val_as_obj(v);
    if (o->type == OBJ_CLOSURE) return (uint32_t)graph_func_id(g, (ObjClosure *)o);
    if (o->type == OBJ_CLASS) return (uint32_t)(g->nf + graph_class_id(g, (ObjClass *)o));
    return 0;
}

/* ---------- 序列化 ---------- */

static void ser_value(Writer *w, ObjGraph *g, Value v) {
    if (val_is_null(v)) { w_u8(w, 0); }
    else if (val_is_bool(v)) { w_u8(w, 1); w_u8(w, val_as_bool(v) ? 1 : 0); }
    else if (val_is_num(v)) {
        /* int 已删除（2026-08-09）：所有数字都是 float/double。整数用 tag 2（紧凑 i64），
           带小数用 tag 3（f64 全精度）——不可一律 (int64_t) 截断（0.5→0 丢精度）。 */
        double d = val_as_float(v);
        int64_t i;
        if (float_as_int(d, &i)) { w_u8(w, 2); w_i64(w, i); }
        else { w_u8(w, 3); w_f64(w, d); }
    }
    else if (val_is_obj(v)) {
        Obj *o = val_as_obj(v);
        if (o->type == OBJ_STRING) {
            ObjString *s = (ObjString *)o;
            w_u8(w, 4);
            w_str(w, s->chars, s->len);
        } else if (o->type == OBJ_CLOSURE) {
            w_u8(w, 5);
            w_u32(w, graph_obj_id(g, v));
        } else if (o->type == OBJ_CLOSURE) {
            w_u8(w, 7); /* ObjClosure 值：序列化为 func id，deser 重建 permanent closure */
            w_u32(w, graph_obj_id(g, v));
        } else if (o->type == OBJ_CLASS) {
            w_u8(w, 6);
            w_u32(w, graph_obj_id(g, v));
        } else {
            w_u8(w, 0);
        }
    } else {
        w_u8(w, 0);
    }
}

static void ser_proto(Writer *w, ObjGraph *g, Proto *p) {
    w_u32(w, (uint32_t)p->count);
    for (size_t i = 0; i < p->count; i++) w_u32(w, p->code[i]);
    w_u32(w, (uint32_t)p->n_constants);
    for (size_t i = 0; i < p->n_constants; i++) ser_value(w, g, p->constants[i]);
    w_u32(w, (uint32_t)p->n_num_consts);
    for (size_t i = 0; i < p->n_num_consts; i++) ser_value(w, g, p->num_consts[i]);
    w_u32(w, (uint32_t)p->n_str_consts);
    for (size_t i = 0; i < p->n_str_consts; i++) ser_value(w, g, p->str_consts[i]);
    w_u32(w, (uint32_t)p->count);
    for (size_t i = 0; i < p->count; i++) w_i32(w, p->lines[i]);
    /* 闭包捕获描述（is_local + index） */
    w_u32(w, (uint32_t)p->n_capvals);
    for (size_t i = 0; i < p->n_capvals; i++) {
        w_u8(w, p->capvals[i].is_local);
        w_u32(w, p->capvals[i].index);
    }
    /* 标识符池（成员名；SSO String；内联序列化）。 */
    w_u32(w, (uint32_t)p->n_ident_names);
    for (size_t i = 0; i < p->n_ident_names; i++) {
        const String *s = &p->ident_names[i];
        w_str(w, str_data(s), str_len(s));
    }
    /* 全局标识符池（全局变量名；与成员池分离，2026-08-15）。 */
    w_u32(w, (uint32_t)p->n_global_ident_names);
    for (size_t i = 0; i < p->n_global_ident_names; i++) {
        const String *s = &p->global_ident_names[i];
        w_str(w, str_data(s), str_len(s));
    }
}

static void ser_func_body(Writer *w, ObjGraph *g, ObjClosure *f) {
    w_str(w, f->proto->name ? f->proto->name->chars : "", f->proto->name ? f->proto->name->len : 0);
    w_u32(w, f->proto->arity);
    w_u8(w, f->proto->is_method); /* 反射 Method.arity 扣隐藏 self 槽用 */
    w_u8(w, f->proto->is_coroutine); /* 协程体（含 yield）标志 */
    w_u32(w, f->proto->max_regs);
    /* module 归属：反射 Method.module / 错误前缀用；空串 = 无模块 */
    w_str(w, f->proto->module ? f->proto->module->chars : "", f->proto->module ? f->proto->module->len : 0);
    ser_proto(w, g, f->proto);
}

typedef struct { Writer *w; ObjGraph *g; } TableSerCtx;

static int ser_table_count_cb(const String *key, Value v, void *u) {
    (void)key; (void)v;
    (*(size_t *)u)++;
    return 0;
}
static int ser_table_entry_cb(const String *key, Value v, void *u) {
    TableSerCtx *c = (TableSerCtx *)u;
    w_str(c->w, str_data(key), str_len(key));
    w_u32(c->w, graph_obj_id(c->g, v));
    return 0;
}
static void ser_table(Writer *w, ObjGraph *g, ObjTable *t) {
    size_t n = 0;
    objtable_iter(t, ser_table_count_cb, &n);
    w_u32(w, (uint32_t)n);
    TableSerCtx c = {w, g};
    objtable_iter(t, ser_table_entry_cb, &c);
}

static void ser_class_body(Writer *w, ObjGraph *g, ObjClass *c) {
    w_str(w, c->name->chars, c->name->len);
    /* module 归属：Type.full_name / Method.module 用；空串 = 无模块 */
    w_str(w, c->module ? c->module->chars : "", c->module ? c->module->len : 0);
    /* superclass */
    if (c->superclass) { w_u32(w, (uint32_t)(g->nf + graph_class_id(g, c->superclass))); }
    else { w_u32(w, 0xFFFFFFFF); }
    /* field_names */
    w_u32(w, (uint32_t)c->n_field_names);
    for (size_t i = 0; i < c->n_field_names; i++) w_str(w, c->field_names[i]->chars, c->field_names[i]->len);
    /* defaults */
    if (c->defaults) { w_u8(w, 1); w_u32(w, (uint32_t)graph_func_id(g, c->defaults)); }
    else { w_u8(w, 0); }
    ser_table(w, g, c->methods);
    ser_table(w, g, c->static_methods);
    ser_table(w, g, c->getters);
    ser_table(w, g, c->setters);
}

int assembly_write(Gc *gc, const char *path, const char *const *module_names, size_t n_modules,
                   const char *const *const *module_exports, const size_t *export_counts,
                   ObjClosure *const *module_inits, ObjClosure *entry) {
    ObjGraph g = {0};
    g.gc = gc;
    for (size_t i = 0; i < n_modules; i++) graph_collect_func(&g, module_inits[i]);
    graph_collect_func(&g, entry);

    Writer w = {0};
    w_bytes(&w, "MICAASM", 8);
    w_u32(&w, 11);   /* version 11: 删 OP_COROUTINE（协程工厂=OP_CLOSURE+proto->is_coroutine）；旧格式须重编 */
    w_u32(&w, (uint32_t)n_modules);
    for (size_t i = 0; i < n_modules; i++) {
        w_str(&w, module_names[i], strlen(module_names[i]));
        w_u32(&w, (uint32_t)export_counts[i]);
        for (size_t j = 0; j < export_counts[i]; j++) {
            w_str(&w, module_exports[i][j], strlen(module_exports[i][j]));
        }
        w_u32(&w, (uint32_t)graph_func_id(&g, module_inits[i]));
    }
    /* entry 可为 NULL（库程序集无入口）：写哨兵 0xFFFFFFFF */
    if (entry) {
        w_u32(&w, (uint32_t)graph_func_id(&g, entry));
    } else {
        w_u32(&w, 0xFFFFFFFF);
    }

    const uint32_t n_objs = (uint32_t)(g.nf + g.nc);
    w_u32(&w, n_objs);
    for (size_t i = 0; i < g.nf; i++) w_u8(&w, 0);          /* func */
    for (size_t i = 0; i < g.nc; i++) w_u8(&w, 1);          /* class */
    for (size_t i = 0; i < g.nf; i++) ser_func_body(&w, &g, g.funcs[i]);
    for (size_t i = 0; i < g.nc; i++) ser_class_body(&w, &g, g.classes[i]);

    FILE *f = fopen(path, "wb");
    if (!f) { free(w.buf); free(g.funcs); free(g.classes); return 1; }
    const size_t written = fwrite(w.buf, 1, w.len, f);
    fclose(f);
    free(w.buf);
    free(g.funcs);
    free(g.classes);
    return (written == w.len) ? 0 : 1;
}

/* ---------- 反序列化 ---------- */

static Value deser_value(Reader *r, Gc *gc, void **objs, uint32_t n_objs);
static Proto *deser_proto(Reader *r, Gc *gc, void **objs, uint32_t n_objs);

static Value deser_value(Reader *r, Gc *gc, void **objs, uint32_t n_objs) {
    const uint8_t tag = r_u8(r);
    switch (tag) {
        case 0: return val_null();
        case 1: return val_bool(r_u8(r) != 0);
        case 2: return val_int(r_i64(r));
        case 3: return val_float(r_f64(r));
        case 4: {
            size_t n = 0;
            char *s = r_str(r, &n);
            if (!s) return val_null();
            Value v = obj_string_val(obj_string_new_permanent(gc, s, n));
            free(s);
            return v;
        }
        case 5: {
            const uint32_t id = r_u32(r);
            ObjClosure *c = (ObjClosure *)OBJ_AT(objs, n_objs, id);
            return c ? obj_closure_val(c) : val_null();
        }
        case 6: {
            const uint32_t id = r_u32(r);
            ObjClass *c = (ObjClass *)OBJ_AT(objs, n_objs, id);
            return c ? obj_class_val(c) : val_null();
        }
        case 7: {
            const uint32_t id = r_u32(r);
            ObjClosure *c = (ObjClosure *)OBJ_AT(objs, n_objs, id);
            return c ? obj_closure_val(c) : val_null();
        }
        default: return val_null();
    }
}

static Proto *deser_proto(Reader *r, Gc *gc, void **objs, uint32_t n_objs) {
    Proto *p = proto_new();
    const uint32_t nc = r_u32(r);
    p->count = nc;
    proto_reserve_pool(p, nc, 0, 0, 0, 0, 0); /* 先预分配 code 区（顺序：count→code→常量计数） */
    for (uint32_t i = 0; i < nc; i++) p->code[i] = r_u32(r);
    /* 三常量区「计数+值」交错读取（ser 格式：nk, nk 个通用, nn, nn 个数, ns, ns 个字符串） */
    const uint32_t nk = r_u32(r);   /* 通用常量数 */
    proto_reserve_pool(p, nc, nk, 0, 0, 0, 0);
    p->n_constants = nk;
    for (uint32_t i = 0; i < nk; i++) p->constants[i] = deser_value(r, gc, objs, n_objs);
    const uint32_t nn = r_u32(r);   /* 数值常量数 */
    proto_reserve_pool(p, nc, nk, nn, 0, 0, 0);
    p->n_num_consts = nn;
    for (uint32_t i = 0; i < nn; i++) p->num_consts[i] = deser_value(r, gc, objs, n_objs);
    const uint32_t ns = r_u32(r);   /* 字符串常量数 */
    proto_reserve_pool(p, nc, nk, nn, ns, 0, 0);
    p->n_str_consts = ns;
    for (uint32_t i = 0; i < ns; i++) p->str_consts[i] = deser_value(r, gc, objs, n_objs);
    const uint32_t nl = r_u32(r);
    /* lines 已由 proto_reserve_pool 按 cap_code ≥ nc 分配（nc==nl，同一 count）；
       直接复用池缓冲，勿再 malloc（覆盖指针会泄漏池分配的缓冲）。 */
    assert((size_t)nl <= p->cap);
    for (uint32_t i = 0; i < nl; i++) p->lines[i] = r_i32(r);
    /* 闭包捕获描述 */
    const uint32_t nu = r_u32(r);
    if (nu > 0) {
        p->capvals = malloc((size_t)nu * sizeof(CapvalDesc));
        assert(p->capvals != NULL);
        p->n_capvals = nu;
        for (uint32_t i = 0; i < nu; i++) {
            p->capvals[i].is_local = r_u8(r);
            p->capvals[i].index = (uint16_t)r_u32(r);
        }
    }
    /* 标识符池：重建 SSO String（proto_add_ident 归一 @ 形式；旧 .mlib 的
       "System.X" 全名也会被压缩成 "@X"，与新格式一致） */
    const uint32_t ni = r_u32(r);
    for (uint32_t i = 0; i < ni; i++) {
        size_t il = 0;
        char *idn = r_str(r, &il);
        proto_add_ident(p, idn, il);
        free(idn);
    }
    /* 全局标识符池（全局变量名；proto_add_global_ident @ 归一） */
    const uint32_t ngi = r_u32(r);
    for (uint32_t i = 0; i < ngi; i++) {
        size_t il = 0;
        char *idn = r_str(r, &il);
        proto_add_global_ident(p, idn, il);
        free(idn);
    }
    return p;
}

static void deser_func_body(Reader *r, Gc *gc, void **objs, uint32_t n_objs, ObjClosure *f) {
    size_t nl = 0;
    char *nm = r_str(r, &nl);
    ObjString *name = nm ? obj_string_new_permanent(gc, nm, nl) : NULL;
    free(nm);
    uint8_t arity = (uint8_t)r_u32(r);
    bool is_method = (bool)r_u8(r);
    bool is_coroutine = (bool)r_u8(r);
    uint8_t max_regs = (uint8_t)r_u32(r);
    size_t ml = 0;
    char *mn = r_str(r, &ml);
    ObjString *mod = (mn && ml > 0) ? obj_string_new_permanent(gc, mn, ml) : NULL;
    free(mn);
    Proto *p = deser_proto(r, gc, objs, n_objs);
    p->name = name;
    p->module = mod; /* module 归属：反射 Method.module / 错误前缀用 */
    p->arity = arity;
    p->is_method = is_method;
    p->is_coroutine = is_coroutine;
    p->max_regs = max_regs;
    f->proto = p;
    f->n_capvals = 0; /* permanent 闭包：无捕获值 */
}

static ObjTable *deser_table(Reader *r, Gc *gc, void **objs, uint32_t n_objs) {
    ObjTable *t = objtable_new_permanent(gc);
    const uint32_t n = r_u32(r);
    for (uint32_t i = 0; i < n; i++) {
        size_t nl = 0;
        char *nm = r_str(r, &nl);
        if (!nm) break;
        const uint32_t hash = fnv_hash(nm, nl);
        const uint32_t mid = r_u32(r);
        ObjClosure *f = (ObjClosure *)OBJ_AT(objs, n_objs, mid);
        if (f) objtable_set_kv(gc, t, hash, nm, nl, obj_closure_val(f));
        free(nm);
    }
    return t;
}

static void deser_class_body(Reader *r, Gc *gc, void **objs, uint32_t n_objs, ObjClass *c) {
    c->gc = gc;
    c->module = NULL;
    c->native_ctor = NULL;
    c->field_index = NULL; /* obj_class_build_field_index 靠 field_index==NULL 判幂等；n_total_fields 由它设 */
    c->last_field_hash = 0;
    c->last_field_chars = NULL;
    c->last_field_idx = 0;
    c->field_cache_valid = false; /* P6 字段单槽缓存：误读垃圾值会返回错误字段下标 */
    size_t nl = 0;
    char *nm = r_str(r, &nl);
    c->name = nm ? obj_string_new_permanent(gc, nm, nl) : NULL;
    free(nm);
    size_t ml = 0;
    char *mn = r_str(r, &ml);
    c->module = (mn && ml > 0) ? obj_string_new_permanent(gc, mn, ml) : NULL;
    free(mn);
    const uint32_t super_id = r_u32(r);
    c->superclass = (super_id == 0xFFFFFFFF) ? NULL : (ObjClass *)OBJ_AT(objs, n_objs, super_id);
    const uint32_t nf = r_u32(r);
    c->field_names = malloc((size_t)nf * sizeof(ObjString *));
    assert(c->field_names != NULL);
    for (uint32_t i = 0; i < nf; i++) {
        size_t fl = 0;
        char *fn = r_str(r, &fl);
        c->field_names[i] = fn ? obj_string_new_permanent(gc, fn, fl) : NULL;
        free(fn);
    }
    c->n_field_names = nf;
    if (r_u8(r)) { const uint32_t did = r_u32(r); c->defaults = (ObjClosure *)OBJ_AT(objs, n_objs, did); }
    else c->defaults = NULL;
    c->methods = deser_table(r, gc, objs, n_objs);
    c->static_methods = deser_table(r, gc, objs, n_objs);
    c->getters = deser_table(r, gc, objs, n_objs);
    c->setters = deser_table(r, gc, objs, n_objs);
    c->has_getters = (objtable_size(c->getters) > 0) ||
                     (c->superclass && c->superclass->has_getters);
    c->has_setters = (objtable_size(c->setters) > 0) ||
                     (c->superclass && c->superclass->has_setters);
}

int assembly_read(Gc *gc, const char *path, char ***out_names, size_t *out_n,
                  char ****out_exports, size_t **out_export_counts,
                  ObjClosure ***out_inits, ObjClosure **out_entry) {
    FILE *f = fopen(path, "rb");
    if (!f) return 1;
    fseek(f, 0, SEEK_END);
    const long sz = ftell(f);
    rewind(f);
    if (sz < 8) { fclose(f); return 1; }
    uint8_t *buf = malloc((size_t)sz);
    if (!buf) { fclose(f); return 1; }
    if (fread(buf, 1, (size_t)sz, f) != (size_t)sz) { free(buf); fclose(f); return 1; }
    fclose(f);

    Reader r = {buf, buf + sz};
    /* 长度与魔数检查：旧写法 `if (r_need(&r,8) && memcmp(...))` 在文件不足 8 字节时
       **跳过魔数校验**（虽然上面的 sz<8 早退挡住了这一例，逻辑仍应写对）。 */
    if (!r_need(&r, 8) || memcmp(r.p, "MICAASM", 8) != 0) { free(buf); return 1; }
    r.p += 8;
    if (r_u32(&r) != 11) { free(buf); return 1; } /* version 11：删 OP_COROUTINE + is_coroutine 序列化（2026-08-15） */

    /* 计数上限：n_modules/n_objs 直接来自文件，未加约束时一个 12 字节的畸形文件
       就能诱使 malloc(4G)（分配失败后是 assert → Release 下 NULL 解引用）。 */
    const uint32_t n_modules = r_u32(&r);
    if (n_modules > 65536) { free(buf); return 1; }
    uint32_t *init_ids = malloc((n_modules ? n_modules : 1) * sizeof(uint32_t));
    char **names = malloc((n_modules ? n_modules : 1) * sizeof(char *));
    size_t *export_counts = malloc((n_modules ? n_modules : 1) * sizeof(size_t));
    char ***exports = malloc((n_modules ? n_modules : 1) * sizeof(char **));
    if (!init_ids || !names || !export_counts || !exports) {
        free(buf); free(init_ids); free(names); free(export_counts); free(exports);
        return 1;
    }
    for (uint32_t i = 0; i < n_modules; i++) {
        size_t nl = 0;
        names[i] = r_str(&r, &nl);
        const uint32_t ne = r_u32(&r);
        export_counts[i] = ne;
        exports[i] = malloc((ne ? ne : 1) * sizeof(char *));
        for (uint32_t j = 0; j < ne; j++) exports[i][j] = r_str(&r, &nl);
        init_ids[i] = r_u32(&r);
    }
    const uint32_t entry_id = r_u32(&r);

    const uint32_t n_objs = r_u32(&r);
    if (n_objs > (1u << 22)) { /* 对象数上限（4M）：防畸形文件诱导超大分配 */
        free(buf); free(init_ids); free(names); free(export_counts);
        for (uint32_t i = 0; i < n_modules; i++) {
            for (uint32_t j = 0; j < export_counts[i]; j++) free(exports[i][j]);
            free(exports[i]);
        }
        free(exports);
        return 1;
    }
    if (n_objs == 0) { /* 空程序（应不会） */
        free(buf); free(init_ids); free(names); free(export_counts);
        for (uint32_t i = 0; i < n_modules; i++) free(exports[i]);
        free(exports);
        *out_names = names; *out_n = 0; *out_inits = NULL; *out_entry = NULL;
        *out_exports = exports; *out_export_counts = export_counts;
        return 0;
    }
    uint8_t *types = malloc(n_objs);
    if (!types) { free(buf); free(init_ids); free(names); free(export_counts); free(exports); return 1; }
    for (uint32_t i = 0; i < n_objs; i++) types[i] = r_u8(&r);

    void **objs = calloc(n_objs, sizeof(void *));
    if (!objs) { free(buf); free(init_ids); free(names); free(export_counts); free(exports); free(types); return 1; }
    /* 先建壳（类型已知），再填数据——引用总能解析到已建指针 */
    for (uint32_t i = 0; i < n_objs; i++) {
        if (types[i] == 0) {
            objs[i] = gc_alloc_permanent(gc, sizeof(ObjClosure), OBJ_CLOSURE);
        } else {
            objs[i] = gc_alloc_permanent(gc, sizeof(ObjClass), OBJ_CLASS);
        }
    }
    for (uint32_t i = 0; i < n_objs; i++) {
        if (types[i] == 0) deser_func_body(&r, gc, objs, n_objs, (ObjClosure *)objs[i]);
        else deser_class_body(&r, gc, objs, n_objs, (ObjClass *)objs[i]);
    }
    /* ⚠️ 字节码校验：**畸形/恶意 .mlib 的唯一防线**。反序列化出的指令流此前完全不检查，
       而 VM 对寄存器/常量池下标是裸访问 → 越界读写（内存安全事件）。这里对每个函数
       校验：寄存器窗口（max_regs + 每条指令的操作数）、常量/标识符池下标、跳转目标、
       CALL 的结果模式与实参窗口。任一项不合法即拒绝整个程序集（返回失败，不执行）。
       同时校验 max_regs 不超过编码上限（PROTO_MAX_REGS）。 */
    for (uint32_t i = 0; i < n_objs; i++) {
        if (types[i] != 0) continue;
        ObjClosure *cl = (ObjClosure *)objs[i];
        if (!cl->proto || !proto_validate(cl->proto)) {
            free(buf);
            free(init_ids);
            free(types);
            free(objs);
            for (uint32_t k = 0; k < n_modules; k++) {
                for (uint32_t j = 0; j < export_counts[k]; j++) free(exports[k][j]);
                free(exports[k]);
            }
            free(exports);
            free(names);
            free(export_counts);
            return 1;
        }
    }
    /* 类字段下标表：所有类的 field_names 已填，统一构建（递归保证父类先建） */
    for (uint32_t i = 0; i < n_objs; i++) {
        if (types[i] == 1) obj_class_build_field_index(gc, (ObjClass *)objs[i]);
    }
    /* has_getters/has_setters 依赖父类标志，而 graph_collect_class 子先父后（子 id < 父 id），
       逐 id 填充时子类读到的是父类壳的 malloc 垃圾值（gc_alloc_permanent 不 memset）。
       按 id 逆序（父先子后）重算两标志；外部父类（跨程序集）在模块加载时已设好。 */
    for (uint32_t i = n_objs; i-- > 0;) {
        if (types[i] == 1) {
            ObjClass *c = (ObjClass *)objs[i];
            c->has_getters = (objtable_size(c->getters) > 0) ||
                             (c->superclass && c->superclass->has_getters);
            c->has_setters = (objtable_size(c->setters) > 0) ||
                             (c->superclass && c->superclass->has_setters);
        }
    }

    ObjClosure **inits = malloc((n_modules ? n_modules : 1) * sizeof(ObjClosure *));
    if (!inits) { free(buf); free(init_ids); free(names); free(export_counts); free(exports); free(types); free(objs); return 1; }
    for (uint32_t i = 0; i < n_modules; i++) inits[i] = (ObjClosure *)objs[init_ids[i]];
    ObjClosure *entry = (entry_id == 0xFFFFFFFF) ? NULL : (ObjClosure *)objs[entry_id];

    free(buf);
    free(init_ids);
    free(types);
    free(objs);
    *out_names = names;
    *out_n = n_modules;
    *out_inits = inits;
    *out_entry = entry;
    *out_exports = exports;
    *out_export_counts = export_counts;
    return 0;
}
