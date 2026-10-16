#ifndef MICA_STR_H
#define MICA_STR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "value.h"

#if defined(_MSC_VER)
#  pragma warning(push)
#  pragma warning(disable : 4201) /* C11 匿名联合 */
#endif

/* 字符串：短字符串(<=MaxSsoSize)内联存储（SSO），长字符串堆分配。按值传递。
   hash = fnv_hash 的 32 位值（MurmurHash3 x86_32，与 ObjString.hash 一致）。 */
typedef struct String {
    union {
        struct {
            char* lstr;
            size_t llen;
        };
        struct {
            char sstr[27];
            uint8_t flag;
            uint32_t hash;
        };
    };
} String;

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

/* 32 位 MurmurHash3（x86_32，种子 0，2026-08-11 起替代 FNV-1 64 位低 32 位）。
   离散度与速度：fmix 雪崩混合 + 每 4 字节一次乘法。改算法须同步
   tools/strhash.py + 重新生成 src/strlit.h 的 H_* 常量。 */
uint32_t fnv_hash(const char *str, size_t len);

String str_new(const char *str, size_t len);
void str_init(String *self, const char *str, size_t len);
void str_free(String *self);

/* "System." → "@" 前缀压缩（运行时规范前缀）：name 以 "System." 开头 → 写 "@"+其余
   （"System.Math.sqrt" → "@Math.sqrt"）。@X 唯一 = System.X，System 保留给标准库。
   只会变短（省 6 字节），out 容量 ≥ len 即可。返回写入长度。 */
MICA_FORCEINLINE size_t str_compress_system(const char *name, size_t len, char *out) {
    if (len >= 7 && memcmp(name, "System.", 7) == 0) {
        out[0] = '@';
        if (len > 7) memcpy(out + 1, name + 7, len - 7);
        return len - 6;
    }
    if (len) memcpy(out, name, len);
    return len;
}

/* "@X" → "System.X" 显示展开（str_compress_system 的逆）：错误消息等用户可见场合用。
   非 "@" 开头的名原样。只会变长（多 6 字节），out 容量须 ≥ len + 6。返回写入长度。 */
MICA_FORCEINLINE size_t str_expand_system(const char *name, size_t len, char *out) {
    if (len >= 1 && name[0] == '@') {
        memcpy(out, "System.", 7);
        if (len > 1) memcpy(out + 7, name + 1, len - 1);
        return len + 6;
    }
    if (len) memcpy(out, name, len);
    return len;
}

/* SSO 容量（27）：短串内联字节数。 */
enum { MaxSsoSize = (int)sizeof(((String *)0)->sstr) };

/* 以下访问器一律接受 String 指针（避免按值传参的 32 字节结构体拷贝）。 */
MICA_FORCEINLINE bool str_is_long(const String *self) {
    return (self->flag & 0x1) == 1;
}

MICA_FORCEINLINE size_t str_len(const String *self) {
    return str_is_long(self) ? self->llen : (size_t)MaxSsoSize - (size_t)(self->flag >> 1);
}

/* 返回指向 NUL 结尾数据的指针（长串指向堆、短串指向 sstr）。接受 String 指针：
   返回池/堆内**稳定**指针（不复制到栈临时），可安全传给跨多次嵌套调用的函数
   （objtable_set_kv 内部 malloc/fnv 会复用栈临时，按值版短串指针会悬垂）。 */
MICA_FORCEINLINE const char *str_data(const String *self) {
    return str_is_long(self) ? self->lstr : self->sstr;
}

bool str_equals(String self, const char *str, size_t len);

/* 深拷贝（长字符串重新分配堆内存）。 */
String str_copy(String target);

#endif /* MICA_STR_H */
