#include "str.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

/* ---------- 32 位 MurmurHash3（x86_32，种子 0） ----------
   离散度远优于 FNV-1（fmix 雪崩混合），且每 4 字节一次乘法（FNV 每字节乘），
   短字符串（标识符/字段名）更快。改算法必须同步 tools/strhash.py 并重新生成
   src/strlit.h 的 H_* 编译期常量（NAME_EQ/STR_EQ 热路径预筛依赖哈希一致）。 */

static inline uint32_t rotl32(uint32_t x, int r) {
    return (x << r) | (x >> (32 - r));
}
static inline uint32_t fmix32(uint32_t h) {
    h ^= h >> 16;
    h *= 0x85ebca6b;
    h ^= h >> 13;
    h *= 0xc2b2ae35;
    h ^= h >> 16;
    return h;
}
static inline uint32_t load32(const char *p) {
    uint32_t v;
    memcpy(&v, p, 4); /* 未对齐安全；MSVC 内联成单 mov */
    return v;
}

uint32_t fnv_hash(const char *str, const size_t len) {
    const char *p = str;
    const char *end = str + (len & ~(size_t)3);
    const uint32_t c1 = 0xcc9e2d51;
    const uint32_t c2 = 0x1b873593;
    uint32_t h1 = 0; /* 种子 0 */
    for (; p < end; p += 4) {
        uint32_t k1 = load32(p);
        k1 *= c1;
        k1 = rotl32(k1, 15);
        k1 *= c2;
        h1 ^= k1;
        h1 = rotl32(h1, 13);
        h1 = h1 * 5 + 0xe6546b64;
    }
    uint32_t k1 = 0;
    switch (len & 3) {
        case 3: k1 ^= (uint32_t)(uint8_t)p[2] << 16; /* fallthrough */
        case 2: k1 ^= (uint32_t)(uint8_t)p[1] << 8;  /* fallthrough */
        case 1: {
            k1 ^= (uint32_t)(uint8_t)p[0];
            k1 *= c1;
            k1 = rotl32(k1, 15);
            k1 *= c2;
            h1 ^= k1;
        }
        default: ;
    } /* len&3==0：无尾字节，k1 保持 0 */
    h1 ^= (uint32_t)len;
    h1 = fmix32(h1);
    return h1;
}

static void str_set_llen(String *self, size_t len) {
    self->llen = len;
}

static void str_set_hash(String *self, uint32_t hash) {
    self->hash = hash;
}

static void str_set_flag(String *self, uint8_t flag) {
    self->flag = flag;
}

static void str_set_lstr(String *self, char *lstr) {
    self->lstr = lstr;
}

String str_new(const char *str, size_t len) {
    assert(str != NULL);
    String self = {0};
    if (len > (size_t)MaxSsoSize) {
        char *lstr = malloc(len + 1);
        assert(lstr != NULL);
        memcpy(lstr, str, len);
        lstr[len] = '\0';
        str_set_lstr(&self, lstr);
        str_set_llen(&self, len);
        str_set_flag(&self, 0x1);
    } else {
        memcpy(self.sstr, str, len);
        if (len < (size_t)MaxSsoSize) {
            self.sstr[len] = '\0';
        }
        str_set_flag(&self, (uint8_t)((MaxSsoSize - (int)len) << 1));
    }
    str_set_hash(&self, fnv_hash(str, len));
    return self;
}

void str_init(String *self, const char *str, size_t len) {
    const String temp = str_new(str, len);
    memcpy(self, &temp, sizeof(String));
}

void str_free(String *self) {
    if (str_is_long(self)) {
        free((char *)self->lstr);
    }
    self->sstr[0] = '\0';
    str_set_flag(self, (uint8_t)(MaxSsoSize << 1));
    str_set_hash(self, 0);
}

String str_copy(const String target) {
    if (!str_is_long(&target)) {
        return target;
    }
    char *lstr = malloc(target.llen + 1);
    assert(lstr != NULL);
    memcpy(lstr, target.lstr, target.llen);
    lstr[target.llen] = '\0';
    String result = {0};
    str_set_lstr(&result, lstr);
    str_set_llen(&result, target.llen);
    str_set_flag(&result, 0x1);
    str_set_hash(&result, target.hash);
    return result;
}

bool str_equals(const String self, const char *str, size_t len) {
    return str_len(&self) == len && memcmp(str_data(&self), str, len) == 0;
}
