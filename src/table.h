#ifndef MICA_TABLE_H
#define MICA_TABLE_H

#include <stdbool.h>
#include <stddef.h>

#include "obj.h"
#include "str.h"

/* GC 感知的哈希表：键为 String（SSO 值类型，短名内联；长名堆 lstr 由表拥有），
   值为 Value。键不是 GC 对象——不参与 trace/写屏障，生命周期与表一致（表销毁时 str_free）。
   全局符号表/类方法表/模块表都用它；键名来自标识符池/编译期，短串占绝大多数。

   结构 = 稀疏桶 + 平行稠密数组（LuaJIT/ObjMap 风格，2026-08-09）：
   - sparse[]：哈希桶 → 稠密下标（TABLE_NULL_ELEM=空、TABLE_TOMB=墓碑）。
   - keys[] / values[]：平行插入序数组。稠密下标在 rehash（只重建 sparse）时稳定、
     cap 增长 realloc 后仍有效 → 可用作 LOAD_GLOBAL/STORE_GLOBAL 的直接索引。
     values 是纯 Value[]（8 字节/项，scale-8 热路径直读，无 ObjEntry 40 字节 scale 计算）。 */

#define TABLE_NULL_ELEM ((size_t)-1)  /* 稀疏桶空 */
#define TABLE_TOMB      ((size_t)-2)  /* 稀疏桶墓碑（remove 标记；稠密用 swap-with-last 保持紧凑） */

struct ObjTable {
    ObjHeader
    Obj *gclist;             /* gray/gray_again 链（容器统一偏移） */
    size_t *sparse;          /* 哈希桶 → 稠密下标（NULL=未初始化） */
    uint32_t sparse_cap;     /* 桶数（< 4G；收窄自 size_t：表结构 80→64 字节） */
    uint32_t sparse_used;    /* 存活 + 墓碑 */
    String *keys;            /* 平行键数组（表拥有，remove/finalize 时 str_free） */
    Value *values;           /* 平行值数组（纯 Value，稠密下标直读） */
    uint32_t count;          /* 存活条目数 */
    uint32_t cap;            /* keys/values 容量 */
};

ObjTable *objtable_new(Gc *gc);
ObjTable *objtable_new_permanent(Gc *gc);
Obj *objtable_as_obj(ObjTable *t);          /* Obj* 强转（trace/finalize 回调、gc_free_permanent 用） */

/* 字符串键插入/更新：已存在（按 hash+len+memcmp 匹配）则更新值并返回 false，否则插入返回 true。
   缺键时 str_new(chars,len) 拷贝进表（SSO 内联或堆 lstr，表拥有）。写屏障只登记值。 */
bool objtable_set_kv(Gc *gc, ObjTable *t, uint32_t hash, const char *chars, size_t len, Value v);
bool objtable_get_kv(const ObjTable *t, uint32_t hash, const char *chars, size_t len, Value *out);
bool objtable_remove_kv(Gc *gc, ObjTable *t, uint32_t hash, const char *chars, size_t len);
size_t objtable_size(const ObjTable *t);

/* 索引化全局：按名返回稠密下标（SIZE_MAX=不存在）。稠密下标在 rehash 时稳定、
   cap 增长后仍有效（只增不删的表永久稳定）——用作 LOAD_GLOBAL/STORE_GLOBAL
   的直接索引，无需版本守卫（对比旧的条目指针方案：table_grow 使指针悬垂需 version）。
   VM 热路径直接读 t->values[idx]（纯 Value[]，scale-8）。 */
size_t objtable_find_dense(const ObjTable *t, uint32_t hash, const char *chars, size_t len);

typedef int (*ObjTableIterCb)(const String *key, Value value, void *user);
void objtable_iter(const ObjTable *t, ObjTableIterCb cb, void *user);

/* 注册 OBJ_TABLE 的 vtable（新对象类型示例：注册一次，gc.c 零改动） */
void table_vtables_init(Gc *gc);

#endif /* MICA_TABLE_H */
