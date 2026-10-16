#include "obj.h"

#include <assert.h>
#include <intrin.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "proto.h"
#include "gc.h"
#include "str.h"
#include "table.h"
#include "state.h"

/* ============================================================
   对象类型：GC 托管的 ObjString / ObjArray / ObjMap /
   ObjNativeClosure / ObjTable / ObjCapVal / ObjClosure。
   - trace 回调只枚举子对象（gc_trace_value/gc_trace_obj）；
   - finalize 只释放内嵌缓冲（array.items、map 表、closure.capvals 等），
     不得解引用其它对象（shutdown 顺序无关）。
   ============================================================ */

/* ---------- ObjString ---------- */

/* 运行期字符串：直接分配（不驻留）。曾用全局驻留表去重——但那把每个唯一字符串
   永久保留（permanent 根），游戏每帧创建的计分/HUD 等唯一串永不回收 → 严重泄漏
   （实测 30 秒增长 200MB）。字符串相等本就按内容（obj_string_eq/value_eq），
   无指针同一性依赖；字符串是普通 GC 对象，无引用即被回收。
   懒哈希（2026-08-11）：构造不预计算 fnv_hash（OBJ_HASHED 位清除），首次访问
   obj_string_hash 时才计算并缓存——避免每次构造付 O(len) 哈希（打印/拼接即弃的串零开销）。 */
ObjString *obj_string_new(Gc *gc, const char *s, size_t len) {
    ObjString *str = gc_alloc(gc, sizeof(ObjString) + len + 1, OBJ_STRING);
    str->len = (uint32_t)len;
    str->hash = 0; /* 未计算（OBJ_HASHED 位由 gc_alloc 清零） */
    memcpy(str->chars, s, len);
    str->chars[len] = '\0';
    return str;
}


ObjString *obj_string_new_permanent(Gc *gc, const char *s, size_t len) {
    ObjString *str = gc_alloc_permanent(gc, sizeof(ObjString) + len + 1, OBJ_STRING);
    str->len = (uint32_t)len;
    str->hash = 0; /* 未计算（OBJ_HASHED 位未置） */
    memcpy(str->chars, s, len);
    str->chars[len] = '\0';
    return str;
}

/* 懒哈希计算并缓存（obj_string_hash 访问器首次访问调用）。 */
uint32_t obj_string_hash_compute(ObjString *s) {
    s->hash = fnv_hash(s->chars, s->len);
    s->flags |= OBJ_HASHED;
    return s->hash;
}

/* obj_string_hash / obj_string_eq 已内联到 obj.h（热路径表探测）。 */

/* ---------- ObjArray ---------- */

ObjArray *obj_array_new(Gc *gc, size_t cap) {
    ObjArray *a = gc_alloc(gc, sizeof(ObjArray), OBJ_ARRAY);
    /* gclist 由 gc_alloc 管理（MARKING 期已头插 gray 链，勿覆盖）；其余字段显式初始化 */
    a->count = 0;
    a->capacity = 0;
    a->items = NULL;
    a->elements_kind = ARR_ELEM_EMPTY;
    if (cap > 0) {
        a->items = malloc(cap * sizeof(Value));
        assert(a->items != NULL);
        a->capacity = cap;
        gc_account_alloc(gc, cap * sizeof(Value));
    }
    return a;
}

ObjArray *obj_array_new_permanent(Gc *gc, size_t cap) {
    ObjArray *a = gc_alloc_permanent(gc, sizeof(ObjArray), OBJ_ARRAY);
    a->count = 0;
    a->capacity = 0;
    a->items = NULL;
    a->elements_kind = ARR_ELEM_EMPTY;
    if (cap > 0) {
        a->items = malloc(cap * sizeof(Value));
        assert(a->items != NULL);
        a->capacity = cap;
    }
    return a;
}

static void array_grow(Gc *gc, ObjArray *a) {
    size_t ncap = a->capacity ? a->capacity * 2 : 4;
    size_t delta = (ncap - a->capacity) * sizeof(Value);
    a->items = realloc(a->items, ncap * sizeof(Value));
    assert(a->items != NULL);
    a->capacity = ncap;
    gc_account_alloc(gc, delta);
}

/* 维护 elements_kind：EMPTY + 首写 → 该类型；同型保持；混型/其它 → ANY（保守）。
   任何写非 number/bool 直接退化 ANY（此后不再回升，保证优化只信稳定 NUM/BOOL）。 */
void obj_array_update_kind(ObjArray *a, Value v) {
    uint8_t nk;
    if (val_is_num(v)) nk = ARR_ELEM_NUM;
    else if (val_is_bool(v)) nk = ARR_ELEM_BOOL;
    else { a->elements_kind = ARR_ELEM_ANY; return; }
    const uint8_t k = a->elements_kind;
    if (k == ARR_ELEM_EMPTY) a->elements_kind = nk;
    else if (k == ARR_ELEM_ANY || k != nk) a->elements_kind = ARR_ELEM_ANY;
}

void obj_array_set(Gc *gc, ObjArray *a, size_t i, Value v) {
    assert(i < a->count);
    a->items[i] = v;
    obj_array_update_kind(a, v);
    if (val_is_obj(v)) gc_record_write(gc, (Obj *)a, v);
}

void obj_array_push(Gc *gc, ObjArray *a, Value v) {
    if (a->count == a->capacity) array_grow(gc, a);
    a->items[a->count++] = v;
    obj_array_update_kind(a, v);
    if (val_is_obj(v)) gc_record_write(gc, (Obj *)a, v);
}

Value obj_array_pop(Gc *gc, ObjArray *a) {
    (void)gc;
    if (a->count == 0) return val_null();
    const size_t i = a->count - 1;
    const Value v = a->items[i];
    a->items[i] = val_null(); /* 清槽：GC 只 trace [0,count)，残留引用越界读回即 UAF */
    a->count = i;
    return v;
}

/* ---------- ObjMap：ObjString* 键 + 稀疏桶 + 稠密复合条目（2026-08-11 深改） ---------- */

#define MAP_INIT_SPARSE 16
#define MAP_INIT_CAP 8
#define MAP_HIGH 70
#define MAP_LOW 50
#define MAP_NULL_ELEM ((size_t)-1)
#define MAP_TOMB ((size_t)-2)

/* 稠密条目数组增长：realloc 后稠密下标仍有效（只 append）。 */
static void map_grow(Gc *gc, ObjMap *m) {
    (void)gc;
    if (m->count < m->cap) return;
    size_t ncap = m->cap ? m->cap * 2 : MAP_INIT_CAP;
    m->entries = realloc(m->entries, ncap * sizeof(ObjMapEntry));
    assert(m->entries != NULL);
    m->cap = ncap;
}

/* 分配稀疏桶（首次插入前）。 */
static void map_init_sparse(Gc *gc, ObjMap *m) {
    (void)gc;
    if (m->sparse != NULL) return;
    m->sparse = malloc(MAP_INIT_SPARSE * sizeof(size_t));
    assert(m->sparse != NULL);
    memset(m->sparse, 0xFF, MAP_INIT_SPARSE * sizeof(size_t));
    m->sparse_cap = MAP_INIT_SPARSE;
}

/* 字符串键查找：稀疏桶 → 稠密下标。键是 ObjString（hash/len/chars 直读）。 */
static size_t map_find_dense(const ObjMap *m, uint32_t hash, const char *chars, size_t len) {
    if (m->sparse == NULL) return MAP_NULL_ELEM;
    for (size_t i = 0; i < m->sparse_cap; i++) {
        const size_t s = m->sparse[(hash + i) % m->sparse_cap];
        if (s == MAP_NULL_ELEM) return MAP_NULL_ELEM;
        if (s == MAP_TOMB) continue;
        const ObjString *k = m->entries[s].key;
        if (obj_string_hash(k) == hash && k->len == len && memcmp(k->chars, chars, len) == 0)
            return s;
    }
    return MAP_NULL_ELEM;
}

/* rehash：只重建稀疏桶（entries 不动）——稠密下标在 rehash 后稳定。 */
static void map_rehash(Gc *gc, ObjMap *m) {
    (void)gc;
    size_t ncap = m->sparse_cap;
    while (m->count * 100 / ncap >= MAP_LOW) ncap *= 2;
    size_t *sparse = malloc(ncap * sizeof(size_t));
    assert(sparse != NULL);
    memset(sparse, 0xFF, ncap * sizeof(size_t));
    for (size_t i = 0; i < m->count; i++) {
        const uint32_t h = obj_string_hash(m->entries[i].key);
        for (size_t j = 0; j < ncap; j++) {
            size_t *slot = &sparse[(h + j) % ncap];
            if (*slot == MAP_NULL_ELEM) {
                *slot = i;
                break;
            }
        }
    }
    free(m->sparse);
    m->sparse = sparse;
    m->sparse_cap = ncap;
    m->sparse_used = m->count;
}

ObjMap *obj_map_new(Gc *gc) {
    ObjMap *m = gc_alloc(gc, sizeof(ObjMap), OBJ_MAP);
    m->sparse = NULL;
    m->sparse_cap = 0;
    m->sparse_used = 0;
    m->entries = NULL;
    m->count = 0;
    m->cap = 0;
    return m;
}

ObjMap *obj_map_new_permanent(Gc *gc) {
    ObjMap *m = gc_alloc_permanent(gc, sizeof(ObjMap), OBJ_MAP);
    m->sparse = NULL;
    m->sparse_cap = 0;
    m->sparse_used = 0;
    m->entries = NULL;
    m->count = 0;
    m->cap = 0;
    return m;
}

/* 核心插入：键已就绪（obj_map_set 复用调用方 ObjString；obj_map_set_kv 先建键再调）。
   已存在按 hash+len+memcmp 命中更新值。键与值都写屏障（map 持有键引用）。 */
static bool obj_map_insert(Gc *gc, ObjMap *m, ObjString *key, Value val) {
    const uint32_t hash = obj_string_hash(key);
    const char *chars = key->chars;
    const size_t len = key->len;
    map_init_sparse(gc, m);
    if (m->sparse_used * 100 / m->sparse_cap >= MAP_HIGH) map_rehash(gc, m);

    size_t tomb = SIZE_MAX;
    size_t slot = 0;
    size_t i;
    for (i = 0; i < m->sparse_cap; i++) {
        slot = (hash + i) % m->sparse_cap;
        const size_t s = m->sparse[slot];
        if (s == MAP_NULL_ELEM) {
            if (tomb != SIZE_MAX) slot = tomb;
            break;
        }
        if (s == MAP_TOMB) {
            if (tomb == SIZE_MAX) tomb = slot;
            continue;
        }
        const ObjString *k = m->entries[s].key;
        if (obj_string_hash(k) == hash && k->len == len && memcmp(k->chars, chars, len) == 0) {
            m->entries[s].value = val; /* 更新已存在键 */
            if (val_is_obj(val)) gc_record_write(gc, (Obj *)m, val);
            return false;
        }
    }

    map_grow(gc, m);
    const size_t idx = m->count;
    m->entries[idx].key = key;
    m->entries[idx].value = val;
    m->sparse[slot] = idx;
    m->count++;
    if (tomb == SIZE_MAX) m->sparse_used++;
    gc_record_write(gc, (Obj *)m, obj_string_val(key)); /* 键写屏障（map 持有键引用） */
    if (val_is_obj(val)) gc_record_write(gc, (Obj *)m, val);
    return true;
}

/* 原始键插入/更新（高效路径，免 Value 包装）：已存在直接更新；缺键 unsafe 窗口内
   建 ObjString 键（键分配可能触发 GC，map 是 C 局部时须防回收）再插入。 */
bool obj_map_set_kv(Gc *gc, ObjMap *m, uint32_t hash, const char *chars, size_t len, Value val) {
    const size_t d = map_find_dense(m, hash, chars, len);
    if (d != MAP_NULL_ELEM) {
        m->entries[d].value = val;
        if (val_is_obj(val)) gc_record_write(gc, (Obj *)m, val);
        return false;
    }
    gc_begin_unsafe(gc);
    ObjString *key = obj_string_new(gc, chars, len);
    const bool created = obj_map_insert(gc, m, key, val);
    gc_end_unsafe(gc);
    return created;
}

/* Value 键便捷路径：键须为 ObjString（语言层 map 键强制字符串，调用方先检查）。
   直接复用调用方 ObjString 作键（不拷贝）。 */
bool obj_map_set(Gc *gc, ObjMap *m, Value key, Value val) {
    assert(val_is_string(key));
    return obj_map_insert(gc, m, (ObjString *)val_as_obj(key), val);
}

/* 字符串键插入/更新（标识符池/运行时名字，raw 键）。 */
bool obj_map_set_by_string(Gc *gc, ObjMap *m, const char *key, size_t key_len, uint32_t hash,
                           Value val) {
    return obj_map_set_kv(gc, m, hash, key, key_len, val);
}

bool obj_map_get_kv(const ObjMap *m, uint32_t hash, const char *chars, size_t len, Value *out) {
    const size_t d = map_find_dense(m, hash, chars, len);
    if (d == MAP_NULL_ELEM) return false;
    if (out) *out = m->entries[d].value;
    return true;
}

bool obj_map_get(const ObjMap *m, Value key, Value *out) {
    assert(val_is_string(key));
    ObjString *ks = (ObjString *)val_as_obj(key);
    return obj_map_get_kv(m, obj_string_hash(ks), ks->chars, ks->len, out);
}

bool obj_map_get_by_string(const ObjMap *m, const char *key, size_t key_len, uint32_t hash,
                           Value *out) {
    return obj_map_get_kv(m, hash, key, key_len, out);
}

bool obj_map_delete(Gc *gc, ObjMap *m, Value key) {
    (void)gc;
    assert(val_is_string(key));
    ObjString *ks = (ObjString *)val_as_obj(key);
    const uint32_t hash = obj_string_hash(ks);
    const char *chars = ks->chars;
    const size_t len = ks->len;
    if (m->sparse == NULL) return false;

    size_t idx = SIZE_MAX;
    size_t slot = 0;
    for (size_t i = 0; i < m->sparse_cap; i++) {
        slot = (hash + i) % m->sparse_cap;
        const size_t s = m->sparse[slot];
        if (s == MAP_NULL_ELEM) return false;
        if (s == MAP_TOMB) continue;
        const ObjString *k = m->entries[s].key;
        if (obj_string_hash(k) == hash && k->len == len && memcmp(k->chars, chars, len) == 0) {
            idx = s;
            break;
        }
    }
    if (idx == SIZE_MAX) return false;

    /* 与末尾存活条目交换，保持稠密数组紧凑（key+value 复合整体搬移）。
       被删 ObjString 键由 GC 回收（map 已不再持有引用），不 str_free。 */
    const size_t last = --m->count;
    if (idx != last) {
        m->entries[idx] = m->entries[last];
        const uint32_t lh = obj_string_hash(m->entries[idx].key);
        for (size_t i = 0; i < m->sparse_cap; i++) {
            const size_t s = m->sparse[(lh + i) % m->sparse_cap];
            if (s != MAP_NULL_ELEM && s != MAP_TOMB && s == last) {
                m->sparse[(lh + i) % m->sparse_cap] = idx;
                break;
            }
        }
    }
    m->sparse[slot] = MAP_TOMB;
    return true;
}

size_t obj_map_size(const ObjMap *m) {
    return m->count;
}

void obj_map_iter(const ObjMap *m, ObjMapIterCb cb, void *user) {
    if (!m || !cb) return;
    for (size_t i = 0; i < m->count; i++) {
        cb(m->entries[i].key, m->entries[i].value, user);
    }
}

/* ---------- ObjNativeClosure ---------- */

ObjNativeClosure *obj_native_closure_new_permanent(Gc *gc, mica_native_fn fn, const char *name,
                                                   uint32_t arity) {
    ObjNativeClosure *n = gc_alloc_permanent(gc, sizeof(ObjNativeClosure), OBJ_NATIVE_CLOSURE);
    n->n_capvals = 0;
    n->func = fn;
    n->name = obj_string_new_permanent(gc, name, strlen(name));
    n->arity = arity;
    n->internal = false;
    n->is_method = false;
    return n;
}

Value obj_native_closure_val(ObjNativeClosure *n) { return val_obj((Obj *)n); }
Value obj_capval_val(ObjCapVal *cap) { return val_obj((Obj *)cap); }

/* ---------- ObjNativeClass / ObjNativeInstance（绑定 C 结构体 userdata，独立字段） ---------- */

ObjNativeClass *obj_native_class_new_permanent(Gc *gc, ObjString *name, ObjString *module,
                                               size_t userdata_size,
                                               mica_native_finalizer finalizer) {
    ObjNativeClass *c = gc_alloc_permanent(gc, sizeof(ObjNativeClass), OBJ_NATIVE_CLASS);
    c->name = name;
    c->module = module;
    c->userdata_size = userdata_size;
    c->finalizer = finalizer;
    c->methods = objtable_new_permanent(gc);
    c->static_methods = objtable_new_permanent(gc);
    c->getters = objtable_new_permanent(gc);
    c->setters = objtable_new_permanent(gc);
    c->has_getters = false;
    c->has_setters = false;
    return c;
}

ObjNativeInstance *obj_native_instance_new(Gc *gc, ObjNativeClass *klass) {
    gc_begin_unsafe(gc);
    ObjNativeInstance *i = gc_alloc(gc, sizeof(ObjNativeInstance) + klass->userdata_size,
                                    OBJ_NATIVE_INSTANCE);
    i->klass = klass;
    memset(i->userdata, 0, klass->userdata_size);
    gc_end_unsafe(gc);
    return i;
}

void *obj_native_instance_data(ObjNativeInstance *inst) { return inst->userdata; }

Value obj_native_class_val(ObjNativeClass *c) { return val_obj((Obj *)c); }
Value obj_native_instance_val(ObjNativeInstance *i) { return val_obj((Obj *)i); }

/* 查原生类实例方法 / 静态方法 / 属性 getter/setter（无继承链，name 直接作表键，不创建新字符串）。 */
ObjClosure *obj_native_class_find_method_kv(ObjNativeClass *klass, uint32_t hash, const char *chars,
                                            size_t len) {
    Value v;
    if (objtable_get_kv(klass->methods, hash, chars, len, &v)) return (ObjClosure *)val_as_obj(v);
    return NULL;
}

Value obj_native_class_find_static_kv(ObjNativeClass *klass, uint32_t hash, const char *chars,
                                      size_t len) {
    Value v;
    if (objtable_get_kv(klass->static_methods, hash, chars, len, &v)) return v;
    return val_null();
}

Value obj_native_class_find_getter_kv(ObjNativeClass *klass, uint32_t hash, const char *chars,
                                      size_t len) {
    Value v;
    if (objtable_get_kv(klass->getters, hash, chars, len, &v)) return v;
    return val_null();
}

Value obj_native_class_find_setter_kv(ObjNativeClass *klass, uint32_t hash, const char *chars,
                                      size_t len) {
    Value v;
    if (objtable_get_kv(klass->setters, hash, chars, len, &v)) return v;
    return val_null();
}

/* ---------- ObjCapVal / ObjClosure ---------- */

/* 新建捕获值（closed 初始态：flags 置 OBJ_CAPVAL_CLOSED，value 为 v）。
   调用方要 open 时清标志 + cap->p 指向栈槽（见 vm.c OP_CLOSURE）。 */
ObjCapVal *obj_capval_new(Gc *gc, Value v) {
    ObjCapVal *cap = gc_alloc(gc, sizeof(ObjCapVal), OBJ_CAPVAL);
    cap->open_next = NULL;
    cap->open_prev = NULL;
    cap->flags |= OBJ_CAPVAL_CLOSED; /* 初始 closed：value 持有值（p/value union 下用标志位判） */
    cap->value = v;
    return cap;
}

/* 关闭捕获值：把 *p（栈槽值）拷入自有 value，置 closed 标志。
   写屏障：老捕获值持有 young 值须登记 remembered set。已 closed 则 no-op。 */
void obj_capval_close(Gc *gc, ObjCapVal *cap) {
    if (cap->flags & OBJ_CAPVAL_CLOSED) return;
    gc_record_write(gc, (Obj *)cap, *cap->p);
    cap->value = *cap->p;
    cap->flags |= OBJ_CAPVAL_CLOSED;
}

void obj_capval_set(Gc *gc, ObjCapVal *cap, Value v) {
    if (!(cap->flags & OBJ_CAPVAL_CLOSED)) {
        *cap->p = v; /* open：直接写栈槽（栈是 GC 根范围，无需写屏障） */
    } else {
        gc_record_write(gc, (Obj *)cap, v);
        cap->value = v;
    }
}

Value obj_capval_get(const ObjCapVal *cap) {
    return (cap->flags & OBJ_CAPVAL_CLOSED) ? cap->value : *cap->p;
}

/* 关闭本帧寄存器范围内所有开放捕获值（帧弹出 / context 终结时调用）。
   遍历 context.opencapval 链，关闭 cap->p 落在 [base, base+max_regs) 的开放捕获值
   （slot 地址唯一，跨帧不冲突）；closed 后从链摘除——值在自有 value 字段，
   由闭包 capvals[] 持有，不再需要 context 保底。 */
void obj_frame_close_capvals(Gc *gc, mica_context *ctx, CallFrame *cf) {
    const Value *lo = cf->base;
    /* 扫描边界 = 帧寄存器区（无独立机器码区） */
    const Value *hi = cf->base + cf->closure->proto->max_regs;
    ObjCapVal *cap = ctx->opencapval;
    while (cap) {
        ObjCapVal *next = cap->open_next;
        if (!(cap->flags & OBJ_CAPVAL_CLOSED) && cap->p >= lo && cap->p < hi) {
            obj_capval_close(gc, cap);
            if (cap->open_prev) cap->open_prev->open_next = cap->open_next;
            else ctx->opencapval = cap->open_next;
            if (cap->open_next) cap->open_next->open_prev = cap->open_prev;
            cap->open_next = cap->open_prev = NULL;
        }
        cap = next;
    }
}

void obj_context_close_all_capvals(Gc *gc, mica_context *ctx) {
    ObjCapVal *cap = ctx->opencapval;
    while (cap) {
        ObjCapVal *next = cap->open_next;
        obj_capval_close(gc, cap);
        if (cap->open_prev) cap->open_prev->open_next = cap->open_next;
        else ctx->opencapval = cap->open_next;
        if (cap->open_next) cap->open_next->open_prev = cap->open_prev;
        cap->open_next = cap->open_prev = NULL;
        cap = next;
    }
}

ObjClosure *obj_closure_new(Gc *gc, Proto *proto, uint8_t n_capvals) {
    ObjClosure *cl = gc_alloc(gc, sizeof(ObjClosure) + (size_t)n_capvals * sizeof(ObjCapVal *),
                              OBJ_CLOSURE);
    cl->proto = proto;
    cl->n_capvals = n_capvals;
    for (uint8_t k = 0; k < n_capvals; k++) cl->capvals[k] = NULL; /* 显式 NULL（原 memset） */
    return cl;
}

/* permanent 闭包：普通函数/方法/顶层函数的值（capvals 空，编译期阻止捕获）。 */
ObjClosure *obj_closure_new_permanent(Gc *gc, Proto *proto) {
    ObjClosure *cl = gc_alloc_permanent(gc, sizeof(ObjClosure), OBJ_CLOSURE);
    cl->proto = proto;
    cl->n_capvals = 0;
    return cl;
}

/* ---------- ObjError ---------- */

ObjError *obj_error_new(Gc *gc, ObjString *message, ObjString *set_name, ObjString *case_name,
                        ObjString **payload_names, Value *payload, size_t n_payload) {
    ObjError *e = gc_alloc(gc, sizeof(ObjError), OBJ_ERROR);
    e->message = message;
    e->set_name = set_name;
    e->case_name = case_name;
    e->payload_names = payload_names;
    e->payload = payload;
    e->n_payload = n_payload;
    return e;
}

Value obj_error_val(ObjError *e) {
    return val_obj((Obj *)e);
}

/* ---------- ObjEnumVal ---------- */

ObjEnumVal *obj_enum_val_new(Gc *gc, ObjString *set_name, ObjString *case_name,
                             Value *payload, size_t n_payload) {
    ObjEnumVal *e = gc_alloc(gc, sizeof(ObjEnumVal), OBJ_ENUM_VAL);
    e->set_name = set_name;
    e->case_name = case_name;
    e->payload = payload;
    e->n_payload = n_payload;
    return e;
}

Value obj_enum_val(ObjEnumVal *e) {
    return val_obj((Obj *)e);
}

/* ---------- ObjClass / ObjInstance ---------- */

ObjClass *obj_class_new_permanent(Gc *gc, ObjString *name) {
    ObjClass *c = gc_alloc_permanent(gc, sizeof(ObjClass), OBJ_CLASS);
    c->gc = gc;
    c->name = name;
    c->module = NULL;
    c->superclass = NULL;
    c->methods = objtable_new_permanent(gc);
    c->static_methods = objtable_new_permanent(gc);
    c->getters = objtable_new_permanent(gc);
    c->setters = objtable_new_permanent(gc);
    c->has_getters = false;
    c->has_setters = false;
    c->field_names = NULL;
    c->n_field_names = 0;
    c->field_index = NULL;
    c->n_total_fields = 0;
    c->last_field_hash = 0;
    c->last_field_chars = NULL;
    c->last_field_idx = 0;
    c->field_cache_valid = false;
    c->defaults = NULL;
    c->native_ctor = NULL;
    return c;
}

ObjInstance *obj_instance_new(Gc *gc, ObjClass *klass) {
    /* 关键不变量：新建对象从创建到存入根的间隙无收集。
       fields 数组随实例一并分配。NaN-boxing 后 gc_alloc 零值 = float 0.0（非 null），
       须显式填 val_null()（未声明字段默认 null 语义）。 */
    gc_begin_unsafe(gc);
    ObjInstance *i = gc_alloc(gc, sizeof(ObjInstance) + klass->n_total_fields * sizeof(Value),
                              OBJ_INSTANCE);
    i->klass = klass;
    for (size_t k = 0, n = klass->n_total_fields; k < n; k++) i->fields[k] = val_null();
    gc_end_unsafe(gc);
    return i;
}

Value obj_class_val(ObjClass *c) {
    return val_obj((Obj *)c);
}

ObjBuiltinMethod *obj_builtin_method_new(Gc *gc, mica_native_fn fn) {
    ObjBuiltinMethod *bm = gc_alloc(gc, sizeof(ObjBuiltinMethod), OBJ_BUILTIN_METHOD);
    bm->fn = fn;
    return bm;
}

ObjBuiltinMethod *obj_builtin_method_new_permanent(Gc *gc, mica_native_fn fn) {
    ObjBuiltinMethod *bm =
        gc_alloc_permanent(gc, sizeof(ObjBuiltinMethod), OBJ_BUILTIN_METHOD);
    bm->fn = fn;
    return bm;
}

Value obj_builtin_method_val(ObjBuiltinMethod *bm) {
    return val_obj((Obj *)bm);
}

ObjWeakRef *obj_weakref_new(Gc *gc, Value target) {
    /* unsafe 窗口：gc_alloc 不触发收集，target（调用方持有的 C 局部）安全。 */
    gc_begin_unsafe(gc);
    ObjWeakRef *r = gc_alloc(gc, sizeof(ObjWeakRef), OBJ_WEAKREF);
    r->target = target;
    gc_add_weakref(gc, r);
    gc_end_unsafe(gc);
    return r;
}

Value obj_weakref_val(ObjWeakRef *r) {
    return val_obj((Obj *)r);
}

/* ---------- 反射对象（构造器已移入 librefl.c：LREFL_* 自定义类型） ---------- */

ObjClosure *obj_class_find_method(ObjClass *klass, ObjString *name) {
    return obj_class_find_method_kv(klass, obj_string_hash(name), name->chars, name->len);
}

ObjClosure *obj_class_find_method_kv(ObjClass *klass, uint32_t hash, const char *chars, size_t len) {
    for (ObjClass *k = klass; k; k = k->superclass) {
        Value v;
        if (objtable_get_kv(k->methods, hash, chars, len, &v)) return (ObjClosure *)val_as_obj(v);
    }
    return NULL;
}

Value obj_class_find_static(ObjClass *klass, ObjString *name) {
    return obj_class_find_static_kv(klass, obj_string_hash(name), name->chars, name->len);
}

Value obj_class_find_static_kv(ObjClass *klass, uint32_t hash, const char *chars, size_t len) {
    for (ObjClass *k = klass; k; k = k->superclass) {
        Value v;
        if (objtable_get_kv(k->static_methods, hash, chars, len, &v)) return v; /* ObjClosure 或 ObjNativeClosure */
    }
    return val_null();
}

Value obj_class_find_getter(ObjClass *klass, ObjString *name) {
    return obj_class_find_getter_kv(klass, obj_string_hash(name), name->chars, name->len);
}

Value obj_class_find_getter_kv(ObjClass *klass, uint32_t hash, const char *chars, size_t len) {
    for (ObjClass *k = klass; k; k = k->superclass) {
        Value v;
        if (objtable_get_kv(k->getters, hash, chars, len, &v)) return v;
    }
    return val_null();
}

Value obj_class_find_setter(ObjClass *klass, ObjString *name) {
    return obj_class_find_setter_kv(klass, obj_string_hash(name), name->chars, name->len);
}

Value obj_class_find_setter_kv(ObjClass *klass, uint32_t hash, const char *chars, size_t len) {
    for (ObjClass *k = klass; k; k = k->superclass) {
        Value v;
        if (objtable_get_kv(k->setters, hash, chars, len, &v)) return v;
    }
    return val_null();
}

bool obj_class_has_field(const ObjClass *klass, ObjString *name) {
    return obj_class_has_field_kv(klass, obj_string_hash(name), name->chars, name->len);
}

bool obj_class_has_field_kv(const ObjClass *klass, uint32_t hash, const char *chars, size_t len) {
    return obj_class_field_index_kv(klass, hash, chars, len) >= 0;
}

/* 复制父类 field_index 条目到子类表（前缀不变量：父类下标与父类一致）。 */
typedef struct { Gc *gc; ObjTable *dst; } FieldIndexCopyCtx;

static int copy_field_index_cb(const String *key, Value v, void *user) {
    FieldIndexCopyCtx *c = (FieldIndexCopyCtx *)user;
    objtable_set_kv(c->gc, c->dst, key->hash, str_data(key), str_len(key), v);
    return 0;
}

/* 构建 field_index：继承前缀布局（父类字段占 [0, parent_total)，本类字段跟在后面）。
   递归确保父类先建（幂等：field_index 非 NULL 即已建），故 codegen resolve_super /
   deser 均可按任意类顺序调用。 */
void obj_class_build_field_index(Gc *gc, ObjClass *klass) {
    if (klass->field_index) return;
    if (klass->superclass) obj_class_build_field_index(gc, klass->superclass);
    const size_t parent_total = klass->superclass ? klass->superclass->n_total_fields : 0;
    klass->n_total_fields = parent_total + klass->n_field_names;
    ObjTable *idx = objtable_new_permanent(gc);
    if (klass->superclass && klass->superclass->field_index) {
        FieldIndexCopyCtx c = {gc, idx};
        objtable_iter(klass->superclass->field_index, copy_field_index_cb, &c);
    }
    for (size_t i = 0; i < klass->n_field_names; i++) {
        ObjString *fn = klass->field_names[i];
        objtable_set_kv(gc, idx, obj_string_hash(fn), fn->chars, fn->len, val_int((int)(parent_total + i)));
    }
    klass->field_index = idx;
}

int obj_class_field_index(const ObjClass *klass, const String *name) {
    return obj_class_field_index_kv(klass, name->hash, str_data(name), str_len(name));
}

int obj_class_field_index_kv(const ObjClass *klass, uint32_t hash, const char *chars, size_t len) {
    if (!klass->field_index) return -1;
    /* 单槽缓存（P6）：同 class 重复查询同名字段（热循环 self.field）命中免哈希表。
       命中条件 hash + chars 指针相等（编译期字段名 String 不可变 → 指针相等即内容相等）。
       字段下标编译期固定、field_index 表构造后不变 → 缓存永不过期。 */
    if (klass->field_cache_valid && klass->last_field_hash == hash && klass->last_field_chars == chars) {
        return klass->last_field_idx;
    }
    Value v;
    int idx = -1;
    if (objtable_get_kv(klass->field_index, hash, chars, len, &v)) idx = (int)(int64_t)val_as_float(v);
    /* 写缓存（ObjClass 是 permanent，非 const 别名）；单线程解释器无并发 */
    ObjClass *k = (ObjClass *)klass;
    k->last_field_hash = hash;
    k->last_field_chars = chars;
    k->last_field_idx = idx;
    k->field_cache_valid = true;
    return idx;
}

Value obj_instance_get_field(const ObjInstance *inst, const String *name) {
    const int idx = obj_class_field_index(inst->klass, name);
    return idx >= 0 ? inst->fields[idx] : val_null();
}

bool obj_instance_set_field(Gc *gc, ObjInstance *inst, const String *name, Value v) {
    const int idx = obj_class_field_index(inst->klass, name);
    if (idx < 0) return false;
    inst->fields[idx] = v;
    if (val_is_obj(v)) gc_record_write(gc, (Obj *)inst, v);
    return true;
}

/* ---------- vtable 注册（trace 只枚举子对象；finalize 只释放内嵌缓冲） ---------- */

static void array_trace(Obj *obj, GcTraceCtx *ctx) {
    ObjArray *a = (ObjArray *)obj;
    for (size_t i = 0; i < a->count; i++) gc_trace_value(ctx, &a->items[i]);
}

static void array_finalize(Obj *obj) {
    free(((ObjArray *)obj)->items);
}

static void map_trace(Obj *obj, GcTraceCtx *ctx) {
    ObjMap *m = (ObjMap *)obj;
    for (size_t i = 0; i < m->count; i++) {
        gc_trace_obj(ctx, (Obj **)&m->entries[i].key);   /* 键是 GC 对象 */
        gc_trace_value(ctx, &m->entries[i].value);
    }
}

static void map_finalize(Obj *obj) {
    ObjMap *m = (ObjMap *)obj;
    free(m->sparse);
    free(m->entries); /* 键是 GC 对象，由 GC 回收，不 str_free */
}

/* 捕获值 trace：closed（OBJ_CAPVAL_CLOSED）追踪自有 value；open 时值在栈槽，
   trace 该槽（闭包 → capval → 栈槽值的追踪链，"闭包当根"）。 */
static void capval_trace(Obj *obj, GcTraceCtx *ctx) {
    ObjCapVal *cap = (ObjCapVal *)obj;
    if (cap->flags & OBJ_CAPVAL_CLOSED) gc_trace_value(ctx, &cap->value);
    else gc_trace_value(ctx, cap->p);
}

static void closure_trace(Obj *obj, GcTraceCtx *ctx) {
    ObjClosure *cl = (ObjClosure *)obj;
    for (size_t i = 0; i < cl->n_capvals; i++) {
        gc_trace_obj(ctx, (Obj **)&cl->capvals[i]);
    }
}

static void closure_finalize(Obj *obj) {
    ObjClosure *cl = (ObjClosure *)obj;
    /* proto 由 permanent 闭包拥有（编译期分配）；lambda 运行时闭包共享 proto 不 free。
       capvals 是柔性数组，随结构体释放，无需单独 free。 */
    if (cl->flags & OBJ_PERMANENT) proto_free(cl->proto);
}

/* 原生闭包 trace：捕获值（capvals）可能引用 Mica 对象，须保活。func 是 C 指针不 trace。 */
static void native_closure_trace(Obj *obj, GcTraceCtx *ctx) {
    ObjNativeClosure *nc = (ObjNativeClosure *)obj;
    for (size_t i = 0; i < nc->n_capvals; i++) {
        gc_trace_value(ctx, &nc->capvals[i]);
    }
}

static void error_trace(Obj *obj, GcTraceCtx *ctx) {
    ObjError *e = (ObjError *)obj;
    gc_trace_obj(ctx, (Obj **)&e->message);
    gc_trace_obj(ctx, (Obj **)&e->set_name);
    gc_trace_obj(ctx, (Obj **)&e->case_name);
    for (size_t i = 0; i < e->n_payload; i++) {
        gc_trace_value(ctx, &e->payload[i]);
    }
    for (size_t i = 0; i < e->n_payload; i++) {
        gc_trace_obj(ctx, (Obj **)&e->payload_names[i]);
    }
}

static void error_finalize(Obj *obj) {
    ObjError *e = (ObjError *)obj;
    free(e->payload);
    free(e->payload_names);
}

static void enum_val_trace(Obj *obj, GcTraceCtx *ctx) {
    ObjEnumVal *e = (ObjEnumVal *)obj;
    gc_trace_obj(ctx, (Obj **)&e->set_name);
    gc_trace_obj(ctx, (Obj **)&e->case_name);
    for (size_t i = 0; i < e->n_payload; i++) {
        gc_trace_value(ctx, &e->payload[i]);
    }
}

static void enum_val_finalize(Obj *obj) {
    ObjEnumVal *e = (ObjEnumVal *)obj;
    free(e->payload);
}

/* ObjClass 为 permanent：子对象（方法表/默认值）都是 permanent，trace 为 no-op（注册 NULL）；
   finalize 只释放内嵌缓冲（field_names 数组；方法表由各自 finalize 处理）。 */
static void class_finalize(Obj *obj) {
    ObjClass *c = (ObjClass *)obj;
    free(c->field_names);
}

static void instance_trace(Obj *obj, GcTraceCtx *ctx) {
    ObjInstance *i = (ObjInstance *)obj;
    gc_trace_obj(ctx, (Obj **)&i->klass);
    for (size_t k = 0, n = i->klass->n_total_fields; k < n; k++)
        gc_trace_value(ctx, &i->fields[k]);
}

/* 原生实例 finalize：调宿主注册的 userdata 终结回调（释放 C 资源）。
   无 trace——userdata 是宿主 C 结构体原始字节，GC 无法追踪（只能经 finalizer 释放）；
   klass 是 permanent 永不回收，无须保活。 */
static void native_instance_finalize(Obj *obj) {
    ObjNativeInstance *i = (ObjNativeInstance *)obj;
    if (i->klass && i->klass->finalizer) i->klass->finalizer(i->userdata);
}

/* mica_context（OBJ_CONTEXT）：per-context 根追踪（替代栈 GcRootRange）——
   每帧寄存器范围 [base, base+max_regs) + opencapval 链（开放捕获值）+ defer。
   **不能改用 stack_top**：嵌套调用返回会压低 top，父帧高寄存器脱离根范围 →
   存活对象被误回收（用后释放）。闲置 context（无帧）不扫栈。 */
static void context_trace(Obj *obj, GcTraceCtx *ctx) {
    mica_context *c = (mica_context *)obj;
    for (int i = 0; i < c->defer_top; i++) gc_trace_value(ctx, &c->defers[i]);
    if (c->coro_closure) gc_trace_obj(ctx, (Obj **)&c->coro_closure);
    for (int i = 0; i < c->n_coro_start_args; i++) gc_trace_value(ctx, &c->coro_start_args[i]);
    if (!val_is_null(c->st_error_value)) gc_trace_value(ctx, &c->st_error_value); /* Phase 2g 保真 */
    /* native 请求挂起（mica_yield/mica_callk）的产出值：旧实现既不 trace 也不写屏障，
       若 native 产出新对象后再分配，该值只被 ctx->yield_value 引用 → 被误回收。 */
    if (!val_is_null(c->yield_value)) gc_trace_value(ctx, &c->yield_value);
    /* 无帧但栈上有活槽（宿主直调 mica_call 的 native 分支：它在 frame_count==0 下把
       函数/self/实参/结果写在 [stack, stack_top)）：这一区间必须进根集，否则宿主传值
       在 native 内部分配触发 minor 时被回收。有帧时由下面的帧寄存器扫描覆盖。 */
    if (c->frame_count == 0 && c->stack_top > c->stack) {
        for (const Value *v = c->stack; v < c->stack_top; v++) gc_trace_value(ctx, v);
    }
    for (FrameBlock *b = c->frame_blocks; b; b = b->next) {
        for (int f = 0; f < b->used; f++) {
            CallFrame *cf = &b->frames[f];
            gc_trace_obj(ctx, (Obj **)&cf->closure); /* 执行中的闭包须存活 */
            /* 每帧寄存器范围 [base, base+max_regs)。**不能改用 stack_top**：嵌套调用返回会
               压低 top，父帧高寄存器脱离根范围 → 存活对象被误回收（用后释放）。
               全量扫描帧寄存器区。 */
            const int hi = (int)cf->closure->proto->max_regs;
            for (int r = 0; r < hi; r++) {
                gc_trace_value(ctx, &cf->base[r]); /* 帧局部/临时值（含标量，gc_trace_value 跳过） */
            }
        }
    }
    /* 活动帧的开放捕获值须存活：即便暂无闭包引用（如 `(()=>x)()` 立即调用后闭包成垃圾），
       帧弹出时 obj_frame_close_capvals 仍要关闭它们；不标记会被 GC 提前回收 → 用后释放。 */
    for (ObjCapVal *cap = c->opencapval; cap; cap = cap->open_next) {
        gc_trace_obj(ctx, (Obj **)&cap);
    }
}

bool obj_context_is_coroutine(const mica_context *ctx) {
    return CTX_IS_CORO(ctx);
}

void obj_context_finalize(Obj *obj) {
    mica_context *c = (mica_context *)obj;
    /* 从 GC contexts 链摘除（协程 context；主 context 是 permanent 不入链，且由宿主显式释放） */
    if (!(c->flags & OBJ_PERMANENT)) {
        gc_remove_context(c->gc, obj);
    }
    /* 关闭全部开放捕获值（不读帧 closure/proto）：清扫期死 context 的帧闭包可能已先被
       free（闭包新于 context、all_obj 链头侧先被扫）；teardown 时永久闭包（含帧引用的顶层
       函数）的 proto 已在 gc_free 前阶段释放——任何"读 cf->closure->proto->max_regs"都是 UAF。
       整个 context 即将释放，全部关闭即正确（保留被存活闭包引用的捕获值）。 */
    obj_context_close_all_capvals(c->gc, c);
    free(c->stack);
    ctx_frame_blocks_free(c);
    free(c->defers);
    free(c->coro_start_args);
    free(c->error_path);
    free(c->st_error_msg);
}

/* 弱引用：trace 不标记 target（弱语义——目标不因 weakref 存活；注册 NULL 即可，
   回收由 gc_clear_weakrefs 处理）。 */

/* 反射对象 trace 已移入 librefl.c（LREFL_* 自定义类型）。 */

void obj_vtables_init(Gc *gc) {
    gc_register_vtable(gc, OBJ_STRING, (ObjVtable){NULL, NULL});   /* 叶子：无 trace/finalize */
    gc_register_vtable(gc, OBJ_ARRAY, (ObjVtable){array_trace, array_finalize});
    gc_register_vtable(gc, OBJ_MAP, (ObjVtable){map_trace, map_finalize});
    gc_register_vtable(gc, OBJ_NATIVE_CLOSURE, (ObjVtable){native_closure_trace, NULL});
    gc_register_vtable(gc, OBJ_CAPVAL, (ObjVtable){capval_trace, NULL});
    gc_register_vtable(gc, OBJ_CLOSURE, (ObjVtable){closure_trace, closure_finalize});
    gc_register_vtable(gc, OBJ_ERROR, (ObjVtable){error_trace, error_finalize});
    gc_register_vtable(gc, OBJ_ENUM_VAL, (ObjVtable){enum_val_trace, enum_val_finalize});
    gc_register_vtable(gc, OBJ_CLASS, (ObjVtable){NULL, class_finalize}); /* permanent：子对象都 permanent，trace 无需 */
    gc_register_vtable(gc, OBJ_INSTANCE, (ObjVtable){instance_trace, NULL});
    gc_register_vtable(gc, OBJ_BUILTIN_METHOD, (ObjVtable){NULL, NULL});
    gc_register_vtable(gc, OBJ_WEAKREF, (ObjVtable){NULL, NULL}); /* 弱语义：target 不 trace */
    gc_register_vtable(gc, OBJ_CONTEXT, (ObjVtable){context_trace, obj_context_finalize});
    gc_register_vtable(gc, OBJ_NATIVE_CLASS, (ObjVtable){NULL, NULL}); /* permanent 描述符（子对象方法表都 permanent） */
    gc_register_vtable(gc, OBJ_NATIVE_INSTANCE, (ObjVtable){NULL, native_instance_finalize}); /* 叶子：仅 finalizer */
    /* 反射自定义类型（LREFL_*）vtable 由 librefl_vtables_init 注册（mica_runtime_new 调用）。 */
}
