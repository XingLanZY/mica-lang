#include "map.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "array.h"
#include "str.h"

/* 稀疏表初始桶数 */
#define InitSize 16

/* 用量达到该百分比触发 rehash */
#define HighWatermark 70

/* rehash 后用量降到该百分比以下 */
#define LowWatermark 50

/* 稀疏槽哨兵：空 */
#define NullElement ((size_t)-1)

/* 稀疏槽哨兵：已删除 */
#define Tombstone ((size_t)-2)

static size_t align_to(size_t n, size_t align) {
    return (n + align - 1) / align * align;
}

/* 稠密数组中的表项：key + 定长 value（紧随其后） */
typedef struct {
    String key;
    uint8_t value[1];
} MapEntry;

struct Map {
    size_t *sparse;             /* 哈希桶 → 稠密下标 */
    size_t capacity;            /* 稀疏表容量 */
    size_t used;                /* 存活条目 + 墓碑数 */
    Array *dense;               /* 稠密数组（元素为 MapEntry） */
    MapValueDeallocator deallocator;
    size_t value_size;
};

static bool match(const MapEntry *entry, const char *key, size_t key_len, uint32_t hash) {
    return entry->key.hash == hash && str_equals(entry->key, key, key_len);
}

static size_t map_value_size(const Map *self) {
    assert(self != NULL);
    return self->value_size;
}

Map *map_new(size_t value_size, MapValueDeallocator deallocator) {
    assert(value_size > 0);

    Map *self = malloc(sizeof(Map));
    assert(self != NULL);

    self->sparse = malloc(sizeof(size_t) * InitSize);
    assert(self->sparse != NULL);
    memset(self->sparse, 0xFF, sizeof(size_t) * InitSize);
    self->capacity = InitSize;
    self->used = 0;

    self->dense = arr_new(align_to(sizeof(String) + value_size, sizeof(size_t)));
    arr_reserve(self->dense, InitSize);

    self->deallocator = deallocator;
    self->value_size = value_size;

    return self;
}

void map_free(Map *self) {
    assert(self != NULL && self->sparse != NULL && self->dense != NULL);
    free(self->sparse);
    const size_t size = arr_size(self->dense);
    for (size_t i = 0; i < size; i++) {
        MapEntry *entry = (MapEntry *)arr_at(self->dense, i);
        str_free(&entry->key);
        if (self->deallocator != NULL) {
            self->deallocator((void *)entry->value);
        }
    }
    arr_free(self->dense);
    free(self);
}

static void rehash(Map *self) {
    assert(self != NULL);

    size_t capacity = self->capacity;
    const size_t live = arr_size(self->dense);
    while (live * 100 / capacity >= LowWatermark) {
        capacity *= 2;
        assert(capacity > self->capacity);
    }

    size_t *sparse = malloc(sizeof(size_t) * capacity);
    assert(sparse != NULL);
    memset(sparse, 0xFF, sizeof(size_t) * capacity);

    for (size_t i = 0; i < self->capacity; i++) {
        if (self->sparse[i] == NullElement || self->sparse[i] == Tombstone) {
            continue;
        }
        const MapEntry *entry = (MapEntry *)arr_at(self->dense, self->sparse[i]);
        const uint32_t hash = entry->key.hash;
        for (size_t j = 0; j < capacity; j++) {
            size_t *index = &sparse[(hash + j) % capacity];
            if (*index == NullElement) {
                *index = self->sparse[i];
                break;
            }
        }
    }
    free(self->sparse);
    self->sparse = sparse;
    self->capacity = capacity;
    self->used = live;
}

void *map_push(Map *self, const char *key) {
    assert(self != NULL && key != NULL);
    if (self->used * 100 / self->capacity >= HighWatermark) {
        rehash(self);
    }
    const size_t key_len = strlen(key);
    const uint32_t hash = fnv_hash(key, key_len);

    for (size_t i = 0; i < self->capacity; i++) {
        const size_t j = (hash + i) % self->capacity;
        size_t *index = &self->sparse[j];
        if (*index == NullElement) {
            *index = arr_size(self->dense);
            MapEntry *entry = (MapEntry *)arr_push(self->dense);
            str_init(&entry->key, key, key_len);
            self->used++;
            return (void *)(uint8_t *)entry->value;
        }
        if (*index == Tombstone) {
            *index = arr_size(self->dense);
            MapEntry *entry = (MapEntry *)arr_push(self->dense);
            str_init(&entry->key, key, key_len);
            return (void *)(uint8_t *)entry->value;
        }
        const MapEntry *entry = (MapEntry *)arr_at(self->dense, *index);
        if (match(entry, key, key_len, hash)) {
            return NULL;
        }
    }
    return NULL;
}

void map_push_v(Map *self, const char *key, const void *value) {
    assert(self != NULL && key != NULL && value != NULL);
    void *dest = map_push(self, key);
    if (dest != NULL) {
        memcpy(dest, value, map_value_size(self));
    }
}

static void get_index_entry(const Map *self, const char *key, size_t key_len,
                            size_t **idx, MapEntry **entry) {
    assert(self != NULL && key != NULL);
    const uint32_t hash = fnv_hash(key, key_len);
    for (size_t i = 0; i < self->capacity; i++) {
        size_t *index = &self->sparse[(hash + i) % self->capacity];
        if (*index == NullElement) {
            return;
        }
        if (*index == Tombstone) {
            continue;
        }
        MapEntry *ent = (MapEntry *)arr_at(self->dense, *index);
        if (match(ent, key, key_len, hash)) {
            *idx = index;
            *entry = ent;
            return;
        }
    }
}

void *map_find(const Map *self, const char *key) {
    return map_find2(self, key, strlen(key));
}

void *map_find2(const Map *self, const char *key, size_t key_len) {
    assert(self != NULL && key != NULL);
    size_t *index;
    MapEntry *entry = NULL;
    get_index_entry(self, key, key_len, &index, &entry);
    return entry == NULL ? NULL : (void *)entry->value;
}

bool map_has(const Map *self, const char *key) {
    return map_has2(self, key, strlen(key));
}

bool map_has2(const Map *self, const char *key, size_t key_len) {
    return map_find2(self, key, key_len) != NULL;
}

void map_erase(Map *self, const char *key) {
    assert(self != NULL && self->sparse != NULL && key != NULL);
    size_t *index;
    MapEntry *entry = NULL;
    get_index_entry(self, key, strlen(key), &index, &entry);
    if (entry == NULL) {
        return;
    }

    /* 与末尾条目交换：把末尾条目搬到被删槽位，保持稠密数组紧凑。 */
    const size_t size = arr_size(self->dense);
    MapEntry *last = (MapEntry *)arr_pop(self->dense);
    for (size_t i = 0; i < self->capacity; i++) {
        const size_t hash = (last->key.hash + i) % self->capacity;
        if (self->sparse[hash] == size - 1) {
            self->sparse[hash] = *index;
            break;
        }
    }
    str_free(&entry->key);
    if (self->deallocator != NULL) {
        self->deallocator((void *)entry->value);
    }
    if ((void *)entry != (void *)last) {
        memcpy((void *)entry, last, arr_element_size(self->dense));
    }
    *index = Tombstone;
}

void map_iter(const Map *self, MapIterCb cb, void *user) {
    if (self == NULL || cb == NULL || self->dense == NULL) {
        return;
    }
    for (size_t i = 0; i < arr_size(self->dense); i++) {
        const MapEntry *entry = (const MapEntry *)arr_at(self->dense, i);
        if (entry == NULL) {
            continue;
        }
        const int stop = cb(str_data(&entry->key), str_len(&entry->key),
                            (void *)entry->value, user);
        if (stop) {
            return;
        }
    }
}
