#ifndef MICA_MAP_H
#define MICA_MAP_H

#include <stdbool.h>
#include <stddef.h>

/* 哈希表：开放寻址（线性探测）+ 稠密数组（保持插入顺序的迭代）。
   键为 C 字符串（按字节哈希，可配合 find2/has2 变体支持二进制键）。
   值为定长元素，按字节拷贝；可提供析构回调释放值资源。 */
typedef struct Map Map;

typedef void (*MapValueDeallocator)(void *value);

Map *map_new(size_t value_size, MapValueDeallocator deallocator);
void map_free(Map *self);

void *map_find(const Map *self, const char *key);
void *map_find2(const Map *self, const char *key, size_t key_len);
bool map_has(const Map *self, const char *key);
bool map_has2(const Map *self, const char *key, size_t key_len);

/* 若 key 不存在则插入并返回新值槽位；若已存在返回 NULL（不覆盖）。 */
void *map_push(Map *self, const char *key);
void map_push_v(Map *self, const char *key, const void *value);

void map_erase(Map *self, const char *key);

/* 遍历：按插入顺序（erase 会与末尾交换）。cb 返回非 0 提前停止。 */
typedef int (*MapIterCb)(const char *key, size_t key_len, void *value, void *user);
void map_iter(const Map *self, MapIterCb cb, void *user);

#endif /* MICA_MAP_H */
