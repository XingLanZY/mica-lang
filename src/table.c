#include "table.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "gc.h"
#include "str.h"

#define TABLE_INIT_SPARSE 16
#define TABLE_INIT_CAP 8
#define TABLE_HIGH 70
#define TABLE_LOW 50

/* struct ObjTable 定义在 table.h（vm 内联访问 + 稠密下标）。 */

/* ---------- 内部 ---------- */

static size_t table_index(size_t hash, size_t cap) {
    return hash % cap;
}

/* 字符串键查找：稀疏桶 → 稠密下标。键在平行 keys 数组（SSO String，hash 字段 = fnv_hash
   低 32 位，与 ObjString.hash 一致），str_data 取稳定指针（短串 sstr / 长串 lstr）。
   探测用 size_t 算术（hash + i 不溢出），保证 i < cap 时覆盖全部槽位。 */
static size_t table_find_dense(const ObjTable *t, uint32_t hash, const char *chars, size_t len) {
    if (t->sparse == NULL) return TABLE_NULL_ELEM;
    for (size_t i = 0; i < t->sparse_cap; i++) {
        const size_t s = t->sparse[table_index((size_t)hash + i, t->sparse_cap)];
        if (s == TABLE_NULL_ELEM) return TABLE_NULL_ELEM;
        if (s == TABLE_TOMB) continue;
        const String *k = &t->keys[s];
        if (k->hash == hash && str_len(k) == len &&
            memcmp(str_data(k), chars, len) == 0) {
            return s;
        }
    }
    return TABLE_NULL_ELEM;
}

/* rehash：只重建稀疏桶（keys/values 不动）——稠密下标在 rehash 后稳定（索引化全局的前提）。 */
static void table_rehash(Gc *gc, ObjTable *t) {
    (void)gc;
    size_t ncap = t->sparse_cap;
    while (t->count * 100 / ncap >= TABLE_LOW) ncap *= 2;
    size_t *sparse = malloc(ncap * sizeof(size_t));
    assert(sparse != NULL);
    memset(sparse, 0xFF, ncap * sizeof(size_t));
    for (size_t i = 0; i < t->count; i++) {
        const uint32_t h = t->keys[i].hash;
        for (size_t j = 0; j < ncap; j++) {
            size_t *slot = &sparse[table_index((size_t)h + j, ncap)];
            if (*slot == TABLE_NULL_ELEM) {
                *slot = i;
                break;
            }
        }
    }
    free(t->sparse);
    t->sparse = sparse;
    t->sparse_cap = ncap;
    t->sparse_used = t->count;
}

/* 平行 keys/values 数组增长：realloc 后下标仍有效（只 append）。 */
static void table_grow(Gc *gc, ObjTable *t) {
    (void)gc;
    if (t->count < t->cap) return;
    size_t ncap = t->cap ? t->cap * 2 : TABLE_INIT_CAP;
    t->keys = realloc(t->keys, ncap * sizeof(String));
    assert(t->keys != NULL);
    t->values = realloc(t->values, ncap * sizeof(Value));
    assert(t->values != NULL);
    t->cap = ncap;
}

/* 分配稀疏桶（首次插入前）。 */
static void table_init_sparse(Gc *gc, ObjTable *t) {
    (void)gc;
    if (t->sparse != NULL) return;
    t->sparse = malloc(TABLE_INIT_SPARSE * sizeof(size_t));
    assert(t->sparse != NULL);
    memset(t->sparse, 0xFF, TABLE_INIT_SPARSE * sizeof(size_t));
    t->sparse_cap = TABLE_INIT_SPARSE;
}

/* ---------- 工厂 ---------- */

ObjTable *objtable_new(Gc *gc) {
    ObjTable *t = gc_alloc(gc, sizeof(ObjTable), OBJ_TABLE);
    t->sparse = NULL;
    t->sparse_cap = 0;
    t->sparse_used = 0;
    t->keys = NULL;
    t->values = NULL;
    t->count = 0;
    t->cap = 0;
    return t;
}

ObjTable *objtable_new_permanent(Gc *gc) {
    ObjTable *t = gc_alloc_permanent(gc, sizeof(ObjTable), OBJ_TABLE);
    t->sparse = NULL;
    t->sparse_cap = 0;
    t->sparse_used = 0;
    t->keys = NULL;
    t->values = NULL;
    t->count = 0;
    t->cap = 0;
    return t;
}

/* ---------- 操作 ---------- */

bool objtable_set_kv(Gc *gc, ObjTable *t, uint32_t hash, const char *chars, size_t len, Value v) {
    table_init_sparse(gc, t);
    if (t->sparse_used * 100 / t->sparse_cap >= TABLE_HIGH) table_rehash(gc, t);

    size_t tomb = SIZE_MAX;
    size_t slot = 0;
    size_t i;
    for (i = 0; i < t->sparse_cap; i++) {
        slot = table_index((size_t)hash + i, t->sparse_cap);
        const size_t s = t->sparse[slot];
        if (s == TABLE_NULL_ELEM) {
            if (tomb != SIZE_MAX) slot = tomb;
            break;
        }
        if (s == TABLE_TOMB) {
            if (tomb == SIZE_MAX) tomb = slot;
            continue;
        }
        const String *k = &t->keys[s];
        if (k->hash == hash && str_len(k) == len &&
            memcmp(str_data(k), chars, len) == 0) {
            t->values[s] = v; /* 更新 */
            if (val_is_obj(v)) gc_record_write(gc, (Obj *)t, v);
            return false;
        }
    }

    table_grow(gc, t);
    const size_t idx = t->count;
    t->keys[idx] = str_new(chars, len); /* SSO 内联，或堆 lstr（表拥有） */
    t->values[idx] = v;
    t->sparse[slot] = idx;
    t->count++;
    if (tomb == SIZE_MAX) t->sparse_used++;
    if (val_is_obj(v)) gc_record_write(gc, (Obj *)t, v);
    return true;
}

bool objtable_get_kv(const ObjTable *t, uint32_t hash, const char *chars, size_t len, Value *out) {
    const size_t d = table_find_dense(t, hash, chars, len);
    if (d == TABLE_NULL_ELEM) return false;
    if (out) *out = t->values[d];
    return true;
}

/* 索引化全局：按名返回稠密下标（不存在返回 SIZE_MAX）。下标在 rehash / cap 增长后仍有效
   ——只增不删的表（globals）下标永久稳定，可作 LOAD_GLOBAL/STORE_GLOBAL 直接索引。 */
size_t objtable_find_dense(const ObjTable *t, uint32_t hash, const char *chars, size_t len) {
    return table_find_dense(t, hash, chars, len);
}

bool objtable_remove_kv(Gc *gc, ObjTable *t, uint32_t hash, const char *chars, size_t len) {
    (void)gc;
    if (t->sparse == NULL) return false;
    size_t idx = SIZE_MAX;
    size_t slot = 0;
    for (size_t i = 0; i < t->sparse_cap; i++) {
        slot = table_index((size_t)hash + i, t->sparse_cap);
        const size_t s = t->sparse[slot];
        if (s == TABLE_NULL_ELEM) return false;
        if (s == TABLE_TOMB) continue;
        const String *k = &t->keys[s];
        if (k->hash == hash && str_len(k) == len &&
            memcmp(str_data(k), chars, len) == 0) {
            idx = s;
            break;
        }
    }
    if (idx == SIZE_MAX) return false;

    str_free(&t->keys[idx]);       /* 释放长键 lstr */
    const size_t last = --t->count;
    if (idx != last) {
        /* 末尾条目搬入被删槽位，保持 keys/values[0..count) 紧凑（删档会改变被搬条目的下标） */
        t->keys[idx] = t->keys[last];
        t->values[idx] = t->values[last];
        const uint32_t lh = t->keys[idx].hash;
        for (size_t i = 0; i < t->sparse_cap; i++) {
            const size_t s = t->sparse[table_index((size_t)lh + i, t->sparse_cap)];
            if (s == last) {
                t->sparse[table_index((size_t)lh + i, t->sparse_cap)] = idx;
                break;
            }
        }
    }
    t->sparse[slot] = TABLE_TOMB;
    return true;
}

size_t objtable_size(const ObjTable *t) {
    return t->count;
}

Obj *objtable_as_obj(ObjTable *t) {
    return (Obj *)t;
}

void objtable_iter(const ObjTable *t, ObjTableIterCb cb, void *user) {
    if (!t || !cb) return;
    for (size_t i = 0; i < t->count; i++) {
        if (cb(&t->keys[i], t->values[i], user)) return;
    }
}

/* ---------- vtable（新对象类型：一次注册，gc.c 零改动） ---------- */

/* 键是 String（值类型，非 GC 对象）——只 trace 值，键由表拥有、finalize 时 str_free。 */
static void table_trace(Obj *obj, GcTraceCtx *ctx) {
    ObjTable *t = (ObjTable *)obj;
    for (size_t i = 0; i < t->count; i++) gc_trace_value(ctx, &t->values[i]);
}

static void table_finalize(Obj *obj) {
    ObjTable *t = (ObjTable *)obj;
    for (size_t i = 0; i < t->count; i++) str_free(&t->keys[i]); /* 释放长键 lstr */
    free(t->sparse);
    free(t->keys);
    free(t->values);
}

void table_vtables_init(Gc *gc) {
    gc_register_vtable(gc, OBJ_TABLE, (ObjVtable){table_trace, table_finalize});
}
