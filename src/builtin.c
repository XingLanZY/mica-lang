/* 内置类型成员方法（array/string/map）与标准库底层 native（System.Builtin）。
   独立模块（高内聚）：
   - vm 的 get_member 经 find_builtin_method 按类型查表分派到这里；
   - 宿主经 install_natives / install_builtin_module 注入全局内建与 System.Builtin 命名空间。
   只依赖 mica.h（不透明类型）+ vm.h（runtime_error/make_error/value_to_cstr）
   + obj/value/gc/table，不触碰 mica_context 内部结构——结构定义完全私有于 mica.c。 */

#include "builtin.h"
#include "state.h"
#include "mica.h"
#include "vm.h"
#include "obj.h"
#include "value.h"
#include "gc.h"
#include "table.h"
#include "strlit.h"

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* 空白字符（string.trim 用）。 */
static bool is_ws(unsigned char c) {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\v' || c == '\f';
}

/* ================= 内置类型成员方法（array / string / map） =================
   这些是绑定在数组/字符串/Map 上的 native 方法；args[0] 恒为接收者（self）。
   get_member 查到方法名时返回 OBJ_BUILTIN_METHOD，VM CALL 把 receiver 插入实参前调用。 */

static uint32_t bm_array_push(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 2 || !val_is_array(args[0])) {
        runtime_error(ctx, NULL, "array.push expects an argument");
        return 0;
    }
    ObjArray *a = (ObjArray *)val_as_obj(args[0]);
    obj_array_push(ctx->gc, a, args[1]);
    res[0] = val_int((int64_t)a->count);
    return 1;
}

static uint32_t bm_array_pop(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    (void)argc;
    if (argc < 1 || !val_is_array(args[0])) {
        runtime_error(ctx, NULL, "array.pop expects no argument");
        return 0;
    }
    ObjArray *a = (ObjArray *)val_as_obj(args[0]);
    res[0] = obj_array_pop(ctx->gc, a);
    return 1;
}

static uint32_t bm_array_join(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 1 || !val_is_array(args[0])) {
        runtime_error(ctx, NULL, "array.join expects a separator string");
        return 0;
    }
    ObjArray *a = (ObjArray *)val_as_obj(args[0]);
    const char *sep = (argc >= 2 && val_is_string(args[1])) ? ((ObjString *)val_as_obj(args[1]))->chars : "";
    const size_t seplen = (argc >= 2 && val_is_string(args[1])) ? ((ObjString *)val_as_obj(args[1]))->len : 0;
    char **parts = malloc((a->count ? a->count : 1) * sizeof(char *));
    size_t *plens = malloc((a->count ? a->count : 1) * sizeof(size_t));
    size_t total = 0;
    for (size_t i = 0; i < a->count; i++) {
        parts[i] = value_to_cstr(obj_array_get(a, i));
        plens[i] = strlen(parts[i]);
        total += plens[i];
        if (i + 1 < a->count) total += seplen;
    }
    char *buf = malloc(total + 1);
    size_t w = 0;
    for (size_t i = 0; i < a->count; i++) {
        memcpy(buf + w, parts[i], plens[i]);
        w += plens[i];
        if (i + 1 < a->count) {
            memcpy(buf + w, sep, seplen);
            w += seplen;
        }
        free(parts[i]);
    }
    buf[w] = '\0';
    res[0] = obj_string_val(obj_string_new(ctx->gc, buf, w));
    free(buf);
    free(parts);
    free(plens);
    return 1;
}

static uint32_t bm_array_clear(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    (void)argc;
    (void)res;
    if (argc < 1 || !val_is_array(args[0])) {
        runtime_error(ctx, NULL, "array.clear expects no argument");
        return 0;
    }
    ObjArray *a = (ObjArray *)val_as_obj(args[0]);
    for (size_t i = 0; i < a->count; i++) a->items[i] = val_null(); /* 清槽防 stale 引用 */
    a->count = 0;
    return 0;
}

static uint32_t bm_array_reverse(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    (void)argc;
    (void)res;
    if (argc < 1 || !val_is_array(args[0])) {
        runtime_error(ctx, NULL, "array.reverse expects no argument");
        return 0;
    }
    ObjArray *a = (ObjArray *)val_as_obj(args[0]);
    for (size_t i = 0, j = a->count; i < j; i++, j--) {
        Value t = obj_array_get(a, i);
        obj_array_set(ctx->gc, a, i, obj_array_get(a, j - 1));
        obj_array_set(ctx->gc, a, j - 1, t);
    }
    return 0;
}

static uint32_t bm_array_contains(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 2 || !val_is_array(args[0])) {
        runtime_error(ctx, NULL, "array.contains expects an argument");
        return 0;
    }
    ObjArray *a = (ObjArray *)val_as_obj(args[0]);
    bool found = false;
    for (size_t i = 0; i < a->count; i++) {
        if (value_eq(obj_array_get(a, i), args[1])) { found = true; break; }
    }
    res[0] = val_bool(found);
    return 1;
}

/* ---------- String ---------- */

static uint32_t bm_string_replace(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 3 || !val_is_string(args[0]) || !val_is_string(args[1]) || !val_is_string(args[2])) {
        runtime_error(ctx, NULL, "string.replace expects two string arguments");
        return 0;
    }
    ObjString *s = (ObjString *)val_as_obj(args[0]);
    ObjString *old = (ObjString *)val_as_obj(args[1]);
    ObjString *neu = (ObjString *)val_as_obj(args[2]);
    if (old->len == 0) { res[0] = args[0]; return 1; }
    size_t n = 0;
    for (size_t i = 0; i + old->len <= s->len;) {
        if (memcmp(s->chars + i, old->chars, old->len) == 0) { n++; i += old->len; } else i++;
    }
    if (n == 0) { res[0] = args[0]; return 1; }
    /* 输出长度：用 int64_t 计算差值，避免 neu->len < old->len 时 uint32 下溢
       （0 - 1 == 0xFFFFFFFF → outlen 巨大 → malloc 失败/超时 → 崩溃）。 */
    const int64_t delta = (int64_t)neu->len - (int64_t)old->len;
    const int64_t total = (int64_t)s->len + (int64_t)n * delta;
    if (total < 0) { runtime_error(ctx, NULL, "string.replace: negative length"); return 0; }
    const size_t outlen = (size_t)total;
    char *buf = malloc(outlen + 1);
    size_t w = 0, i = 0;
    while (i < s->len) {
        if (i + old->len <= s->len && memcmp(s->chars + i, old->chars, old->len) == 0) {
            memcpy(buf + w, neu->chars, neu->len);
            w += neu->len;
            i += old->len;
        } else {
            buf[w++] = s->chars[i++];
        }
    }
    buf[w] = '\0';
    res[0] = obj_string_val(obj_string_new(ctx->gc, buf, w));
    free(buf);
    return 1;
}

static uint32_t bm_string_trim(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 1 || !val_is_string(args[0])) {
        runtime_error(ctx, NULL, "string.trim expects no argument");
        return 0;
    }
    ObjString *s = (ObjString *)val_as_obj(args[0]);
    size_t b = 0, e = s->len;
    while (b < e && is_ws((unsigned char)s->chars[b])) b++;
    while (e > b && is_ws((unsigned char)s->chars[e - 1])) e--;
    res[0] = obj_string_val(obj_string_new(ctx->gc, s->chars + b, e - b));
    return 1;
}

static uint32_t bm_string_starts_with(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 2 || !val_is_string(args[0]) || !val_is_string(args[1])) {
        runtime_error(ctx, NULL, "string.starts_with expects a string argument");
        return 0;
    }
    ObjString *s = (ObjString *)val_as_obj(args[0]);
    ObjString *p = (ObjString *)val_as_obj(args[1]);
    res[0] = val_bool(p->len <= s->len && memcmp(s->chars, p->chars, p->len) == 0);
    return 1;
}

static uint32_t bm_string_ends_with(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 2 || !val_is_string(args[0]) || !val_is_string(args[1])) {
        runtime_error(ctx, NULL, "string.ends_with expects a string argument");
        return 0;
    }
    ObjString *s = (ObjString *)val_as_obj(args[0]);
    ObjString *p = (ObjString *)val_as_obj(args[1]);
    res[0] = val_bool(p->len <= s->len && memcmp(s->chars + s->len - p->len, p->chars, p->len) == 0);
    return 1;
}

static uint32_t bm_string_contains(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 2 || !val_is_string(args[0]) || !val_is_string(args[1])) {
        runtime_error(ctx, NULL, "string.contains expects a string argument");
        return 0;
    }
    ObjString *s = (ObjString *)val_as_obj(args[0]);
    ObjString *p = (ObjString *)val_as_obj(args[1]);
    bool found = false;
    if (p->len == 0) found = true;
    else for (size_t i = 0; i + p->len <= s->len; i++) {
        if (memcmp(s->chars + i, p->chars, p->len) == 0) { found = true; break; }
    }
    res[0] = val_bool(found);
    return 1;
}

static uint32_t bm_string_to_upper(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 1 || !val_is_string(args[0])) {
        runtime_error(ctx, NULL, "string.to_upper expects no argument");
        return 0;
    }
    ObjString *s = (ObjString *)val_as_obj(args[0]);
    char *buf = malloc(s->len + 1);
    for (size_t i = 0; i < s->len; i++) {
        unsigned char c = (unsigned char)s->chars[i];
        buf[i] = (c >= 'a' && c <= 'z') ? (char)(c - 32) : (char)c;
    }
    buf[s->len] = '\0';
    res[0] = obj_string_val(obj_string_new(ctx->gc, buf, s->len));
    free(buf);
    return 1;
}

static uint32_t bm_string_to_lower(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 1 || !val_is_string(args[0])) {
        runtime_error(ctx, NULL, "string.to_lower expects no argument");
        return 0;
    }
    ObjString *s = (ObjString *)val_as_obj(args[0]);
    char *buf = malloc(s->len + 1);
    for (size_t i = 0; i < s->len; i++) {
        unsigned char c = (unsigned char)s->chars[i];
        buf[i] = (c >= 'A' && c <= 'Z') ? (char)(c + 32) : (char)c;
    }
    buf[s->len] = '\0';
    res[0] = obj_string_val(obj_string_new(ctx->gc, buf, s->len));
    free(buf);
    return 1;
}

static uint32_t bm_string_substring(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 2 || !val_is_string(args[0]) || !val_is_num(args[1])) {
        runtime_error(ctx, NULL, "string.substring expects int start [, int end]");
        return 0;
    }
    ObjString *s = (ObjString *)val_as_obj(args[0]);
    int64_t start = (int64_t)val_as_float(args[1]);
    int64_t end = (argc >= 3 && val_is_num(args[2])) ? (int64_t)val_as_float(args[2]) : (int64_t)s->len;
    if (start < 0) start = 0;
    if (end < 0) end = 0;
    if (end > (int64_t)s->len) end = (int64_t)s->len;
    if (start > end) start = end;
    res[0] = obj_string_val(obj_string_new(ctx->gc, s->chars + start, (size_t)(end - start)));
    return 1;
}

static uint32_t bm_string_index_of(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 2 || !val_is_string(args[0]) || !val_is_string(args[1])) {
        runtime_error(ctx, NULL, "string.index_of expects a string argument");
        return 0;
    }
    ObjString *s = (ObjString *)val_as_obj(args[0]);
    ObjString *p = (ObjString *)val_as_obj(args[1]);
    if (p->len == 0) { res[0] = val_int(0); return 1; }
    for (size_t i = 0; i + p->len <= s->len; i++) {
        if (memcmp(s->chars + i, p->chars, p->len) == 0) { res[0] = val_int((int64_t)i); return 1; }
    }
    res[0] = val_int(-1);
    return 1;
}

static uint32_t bm_string_split(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 2 || !val_is_string(args[0]) || !val_is_string(args[1])) {
        runtime_error(ctx, NULL, "string.split expects a separator string");
        return 0;
    }
    ObjString *s = (ObjString *)val_as_obj(args[0]);
    ObjString *sep = (ObjString *)val_as_obj(args[1]);
    ObjArray *a = obj_array_new(ctx->gc, 8);
    gc_begin_unsafe(ctx->gc); /* a 是 C 局部；obj_string_new 触发 GC 会回收它（split 压力测试暴露） */
    if (sep->len == 0) {
        for (size_t i = 0; i < s->len; i++) {
            obj_array_push(ctx->gc, a, obj_string_val(obj_string_new(ctx->gc, s->chars + i, 1)));
        }
    } else {
        size_t start = 0;
        for (size_t i = 0; i + sep->len <= s->len; i++) {
            if (memcmp(s->chars + i, sep->chars, sep->len) == 0) {
                obj_array_push(ctx->gc, a, obj_string_val(obj_string_new(ctx->gc, s->chars + start, i - start)));
                i += sep->len - 1;
                start = i + 1;
            }
        }
        obj_array_push(ctx->gc, a, obj_string_val(obj_string_new(ctx->gc, s->chars + start, s->len - start)));
    }
    gc_end_unsafe(ctx->gc);
    res[0] = obj_array_val(a);
    return 1;
}

static uint32_t bm_string_repeat(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 2 || !val_is_string(args[0]) || !val_is_num(args[1])) {
        runtime_error(ctx, NULL, "string.repeat expects an int count");
        return 0;
    }
    ObjString *s = (ObjString *)val_as_obj(args[0]);
    const int64_t n = (int64_t)val_as_float(args[1]);
    if (n <= 0) { res[0] = obj_string_val(obj_string_new(ctx->gc, "", 0)); return 1; }
    if (s->len > 0 && (uint64_t)n > (uint64_t)SIZE_MAX / s->len) {
        runtime_error(ctx, NULL, "string too long");
        return 0;
    }
    const size_t total = (size_t)n * s->len;
    char *buf = malloc(total + 1);
    for (int64_t i = 0; i < n; i++) memcpy(buf + (size_t)i * s->len, s->chars, s->len);
    buf[total] = '\0';
    res[0] = obj_string_val(obj_string_new(ctx->gc, buf, total));
    free(buf);
    return 1;
}

/* ---------- Map ---------- */

typedef struct { Gc *gc; ObjArray *arr; } ArrBuild;

/* ObjMap 键为 ObjString*：keys() 收集时直接推入键值（键已是 GC ObjString）。 */
static void iter_key(ObjString *key, Value v, void *u) {
    (void)v; ArrBuild *b = (ArrBuild *)u;
    obj_array_push(b->gc, b->arr, obj_string_val(key));
}
static void iter_value(ObjString *key, Value v, void *u) {
    (void)key; ArrBuild *b = (ArrBuild *)u;
    obj_array_push(b->gc, b->arr, v);
}

static bool map_key_checked(mica_context *ctx, Value key) {
    if (!val_is_string(key)) {
        runtime_error(ctx, NULL, "map key must be a string");
        return false;
    }
    return true;
}

static uint32_t bm_map_has(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 2 || !val_is_map(args[0])) {
        runtime_error(ctx, NULL, "map.has expects a key");
        return 0;
    }
    if (!map_key_checked(ctx, args[1])) return 0;
    ObjString *ks = (ObjString *)val_as_obj(args[1]);
    Value out;
    res[0] = val_bool(obj_map_get_kv((ObjMap *)val_as_obj(args[0]), obj_string_hash(ks), ks->chars, ks->len, &out));
    return 1;
}

static uint32_t bm_map_get(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 2 || !val_is_map(args[0])) {
        runtime_error(ctx, NULL, "map.get expects a key");
        return 0;
    }
    if (!map_key_checked(ctx, args[1])) return 0;
    ObjString *ks = (ObjString *)val_as_obj(args[1]);
    Value out;
    res[0] = obj_map_get_kv((ObjMap *)val_as_obj(args[0]), obj_string_hash(ks), ks->chars, ks->len, &out)
                 ? out
                 : val_null();
    return 1;
}

static uint32_t bm_map_remove(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 2 || !val_is_map(args[0])) {
        runtime_error(ctx, NULL, "map.remove expects a key");
        return 0;
    }
    if (!map_key_checked(ctx, args[1])) return 0;
    res[0] = val_bool(obj_map_delete(ctx->gc, (ObjMap *)val_as_obj(args[0]), args[1]));
    return 1;
}

static uint32_t bm_map_size(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 1 || !val_is_map(args[0])) {
        runtime_error(ctx, NULL, "map.size expects no argument");
        return 0;
    }
    res[0] = val_int((int64_t)obj_map_size((ObjMap *)val_as_obj(args[0])));
    return 1;
}

static uint32_t bm_map_clear(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    (void)argc;
    (void)res;
    if (argc < 1 || !val_is_map(args[0])) {
        runtime_error(ctx, NULL, "map.clear expects no argument");
        return 0;
    }
    ObjMap *m = (ObjMap *)val_as_obj(args[0]);
    ArrBuild b = {ctx->gc, obj_array_new(ctx->gc, obj_map_size(m))};
    gc_begin_unsafe(ctx->gc); /* 迭代期间 b.arr 是 C 局部（未入根）；obj_array_push 只 realloc 不收集，unsafe 是防御 */
    obj_map_iter(m, iter_key, &b);
    gc_end_unsafe(ctx->gc);
    for (size_t i = 0; i < obj_array_count(b.arr); i++) {
        obj_map_delete(ctx->gc, m, obj_array_get(b.arr, i));
    }
    return 0;
}

static uint32_t bm_map_keys(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 1 || !val_is_map(args[0])) {
        runtime_error(ctx, NULL, "map.keys expects no argument");
        return 0;
    }
    ArrBuild b = {ctx->gc, obj_array_new(ctx->gc, obj_map_size((ObjMap *)val_as_obj(args[0])))};
    gc_begin_unsafe(ctx->gc); /* 迭代期间 b.arr 是 C 局部（未入根）；obj_array_push 只 realloc 不收集，unsafe 是防御 */
    obj_map_iter((ObjMap *)val_as_obj(args[0]), iter_key, &b);
    gc_end_unsafe(ctx->gc);
    res[0] = obj_array_val(b.arr);
    return 1;
}

static uint32_t bm_map_values(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    if (argc < 1 || !val_is_map(args[0])) {
        runtime_error(ctx, NULL, "map.values expects no argument");
        return 0;
    }
    ArrBuild b = {ctx->gc, obj_array_new(ctx->gc, obj_map_size((ObjMap *)val_as_obj(args[0])))};
    gc_begin_unsafe(ctx->gc); /* iter_value push 到 C 局部数组，须防 GC 回收 */
    obj_map_iter((ObjMap *)val_as_obj(args[0]), iter_value, &b);
    gc_end_unsafe(ctx->gc);
    res[0] = obj_array_val(b.arr);
    return 1;
}

/* ---------- 方法表 ---------- */

typedef struct {
    const char *name; /* C 字面量（表行里与 hash/len 并列，勿漂移） */
    uint32_t len;     /* 编译期常量 = sizeof(name)-1 */
    uint32_t hash;    /* 编译期常量 = fnv_hash(name, len)（strlit.h） */
    mica_native_fn fn;
} BuiltinMethodEntry;

/* WeakRef 内置方法：args[0]=weakref 接收者。 */
static uint32_t bm_weakref_get(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    (void)ctx;
    (void)argc;
    res[0] = ((ObjWeakRef *)val_as_obj(args[0]))->target; /* 目标或 null（已回收） */
    return 1;
}

static uint32_t bm_weakref_is_valid(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    (void)ctx;
    (void)argc;
    res[0] = val_bool(!val_is_null(((ObjWeakRef *)val_as_obj(args[0]))->target));
    return 1;
}

static const BuiltinMethodEntry k_array_methods[] = {
    {"push", 4, H_PUSH, bm_array_push}, {"pop", 3, H_POP, bm_array_pop},
    {"join", 4, H_JOIN, bm_array_join}, {"clear", 5, H_CLEAR, bm_array_clear},
    {"reverse", 7, H_REVERSE, bm_array_reverse}, {"contains", 8, H_CONTAINS, bm_array_contains},
};
static const BuiltinMethodEntry k_string_methods[] = {
    {"replace", 7, H_REPLACE, bm_string_replace}, {"trim", 4, H_TRIM, bm_string_trim},
    {"starts_with", 11, H_STARTS_WITH, bm_string_starts_with},
    {"ends_with", 9, H_ENDS_WITH, bm_string_ends_with},
    {"contains", 8, H_CONTAINS, bm_string_contains}, {"to_upper", 8, H_TO_UPPER, bm_string_to_upper},
    {"to_lower", 8, H_TO_LOWER, bm_string_to_lower}, {"substring", 9, H_SUBSTRING, bm_string_substring},
    {"index_of", 8, H_INDEX_OF, bm_string_index_of}, {"split", 5, H_SPLIT, bm_string_split},
    {"repeat", 6, H_REPEAT, bm_string_repeat},
};
static const BuiltinMethodEntry k_map_methods[] = {
    {"has", 3, H_HAS, bm_map_has}, {"get", 3, H_GET, bm_map_get},
    {"remove", 6, H_REMOVE, bm_map_remove}, {"size", 4, H_SIZE, bm_map_size},
    {"clear", 5, H_CLEAR, bm_map_clear}, {"keys", 4, H_KEYS, bm_map_keys},
    {"values", 6, H_VALUES, bm_map_values},
};
static const BuiltinMethodEntry k_weakref_methods[] = {
    {"get", 3, H_GET, bm_weakref_get}, {"is_valid", 8, H_IS_VALID, bm_weakref_is_valid},
};

/* 反射对象方法（bm_refl / k_refl 方法表）与 typeof/assembly 内建已移入 librefl.c。 */


/* 按内置类型查成员方法（get_member 分派用）。
   名字匹配：先比 hash+len（strlit.h 编译期常量）快速拒绝，命中再 memcmp 内容。 */
mica_native_fn find_builtin_method(uint32_t type, ObjString *name) {
    return find_builtin_method_kv(type, obj_string_hash(name), name->chars, name->len);
}

mica_native_fn find_builtin_method_kv(uint32_t type, uint32_t hash, const char *chars, size_t len) {
    const BuiltinMethodEntry *tbl;
    size_t n;
    switch (type) {
        case OBJ_ARRAY: tbl = k_array_methods; n = sizeof(k_array_methods) / sizeof(k_array_methods[0]); break;
        case OBJ_STRING: tbl = k_string_methods; n = sizeof(k_string_methods) / sizeof(k_string_methods[0]); break;
        case OBJ_MAP: tbl = k_map_methods; n = sizeof(k_map_methods) / sizeof(k_map_methods[0]); break;
        case OBJ_WEAKREF: tbl = k_weakref_methods; n = sizeof(k_weakref_methods) / sizeof(k_weakref_methods[0]); break;
        default: return NULL;
    }
    for (size_t i = 0; i < n; i++) {
        const BuiltinMethodEntry *e = &tbl[i];
        if (e->len == len && e->hash == hash && memcmp(e->name, chars, len) == 0) {
            return e->fn;
        }
    }
    return NULL;
}

/* ================= 标准库底层 native（System.Builtin 注入） ================= */

/* ---------- 内建 ---------- */

static uint32_t native_print(mica_context *ctx, mica_value *args, uint32_t argc,
                             mica_value *results) {
    (void)ctx;
    (void)results;
    for (uint32_t i = 0; i < argc; i++) {
        if (i) fputs(" ", stdout);
        value_print(stdout, val_unwrap(args[i]));
    }
    fputc('\n', stdout);
    return 0;
}

/* ---------- 标准库文件 IO natives（IO 模块包装为 IO.read_file 等） ---------- */

static char *str_path_copy(ObjString *s) {
    char *buf = malloc(s->len + 1);
    if (!buf) return NULL;
    memcpy(buf, s->chars, s->len);
    buf[s->len] = '\0';
    return buf;
}

static uint32_t native_read_file(mica_context *ctx, mica_value *args, uint32_t argc, mica_value *res) {
    if (argc < 1 || !val_is_string(val_unwrap(args[0]))) return 0;
    ObjString *path = (ObjString *)val_as_obj(val_unwrap(args[0]));
    char *pb = str_path_copy(path);
    if (!pb) return 0;
    FILE *f = fopen(pb, "rb");
    if (!f) { free(pb); res[0] = val_wrap(make_error(ctx, "cannot open file")); return 1; }
    fseek(f, 0, SEEK_END);
    const long sz = ftell(f);
    rewind(f);
    char *buf = malloc((sz > 0) ? (size_t)sz : 1);
    const size_t rd = fread(buf, 1, (size_t)(sz > 0 ? sz : 0), f);
    fclose(f);
    res[0] = val_wrap(obj_string_val(obj_string_new(ctx->gc, buf, rd)));
    free(buf);
    free(pb);
    return 1;
}

static uint32_t native_write_file(mica_context *ctx, mica_value *args, uint32_t argc, mica_value *res) {
    (void)ctx;
    (void)res;
    if (argc < 2 || !val_is_string(val_unwrap(args[0])) || !val_is_string(val_unwrap(args[1])))
        return 0;
    ObjString *path = (ObjString *)val_as_obj(val_unwrap(args[0]));
    ObjString *content = (ObjString *)val_as_obj(val_unwrap(args[1]));
    char *pb = str_path_copy(path);
    if (!pb) return 0;
    FILE *f = fopen(pb, "wb");
    if (!f) { free(pb); return 0; }
    fwrite(content->chars, 1, content->len, f);
    fclose(f);
    free(pb);
    return 0;
}

static uint32_t native_append_file(mica_context *ctx, mica_value *args, uint32_t argc, mica_value *res) {
    (void)ctx;
    (void)res;
    if (argc < 2 || !val_is_string(val_unwrap(args[0])) || !val_is_string(val_unwrap(args[1])))
        return 0;
    ObjString *path = (ObjString *)val_as_obj(val_unwrap(args[0]));
    ObjString *content = (ObjString *)val_as_obj(val_unwrap(args[1]));
    char *pb = str_path_copy(path);
    if (!pb) return 0;
    FILE *f = fopen(pb, "ab");
    if (!f) { free(pb); return 0; }
    fwrite(content->chars, 1, content->len, f);
    fclose(f);
    free(pb);
    return 0;
}

static uint32_t native_file_exists(mica_context *ctx, mica_value *args, uint32_t argc, mica_value *res) {
    (void)ctx;
    if (argc < 1 || !val_is_string(val_unwrap(args[0]))) { res[0] = val_wrap(val_false()); return 1; }
    ObjString *path = (ObjString *)val_as_obj(val_unwrap(args[0]));
    char *pb = str_path_copy(path);
    bool ok = false;
    if (pb) {
        FILE *f = fopen(pb, "rb");
        if (f) { fclose(f); ok = true; }
        free(pb);
    }
    res[0] = val_wrap(val_bool(ok));
    return 1;
}

static uint32_t native_read_lines(mica_context *ctx, mica_value *args, uint32_t argc, mica_value *res) {
    if (argc < 1 || !val_is_string(val_unwrap(args[0]))) return 0;
    ObjString *path = (ObjString *)val_as_obj(val_unwrap(args[0]));
    char *pb = str_path_copy(path);
    if (!pb) return 0;
    FILE *f = fopen(pb, "rb");
    if (!f) { free(pb); res[0] = val_wrap(make_error(ctx, "cannot open file")); return 1; }
    /* 读全部，按 \n 切分行（去掉 \r） */
    fseek(f, 0, SEEK_END);
    const long sz = ftell(f);
    rewind(f);
    char *buf = malloc((sz > 0) ? (size_t)sz : 1);
    const size_t rd = fread(buf, 1, (size_t)(sz > 0 ? sz : 0), f);
    fclose(f);
    ObjArray *a = obj_array_new(ctx->gc, 8);
    gc_begin_unsafe(ctx->gc); /* a 是 C 局部；obj_string_new 触发 GC 会回收它 */
    size_t start = 0;
    for (size_t i = 0; i <= rd; i++) {
        if (i == rd || buf[i] == '\n') {
            /* 文件以 \n 结尾时不产生额外空段 */
            if (!(i == rd && start == rd)) {
                size_t end = i;
                if (end > start && buf[end - 1] == '\r') end--;
                obj_array_push(ctx->gc, a,
                               obj_string_val(obj_string_new(ctx->gc, buf + start, end - start)));
            }
            start = i + 1;
        }
    }
    gc_end_unsafe(ctx->gc);
    free(buf);
    free(pb);
    res[0] = val_wrap(obj_array_val(a));
    return 1;
}

void install_builtin_module(Gc *gc, ObjTable *globals); /* 前向声明 */

void install_natives(Gc *gc, ObjTable *globals) {
    ObjString *kprint = obj_string_new_permanent(gc, "print", 5);
    Value dummy;
    if (!objtable_get_kv(globals, obj_string_hash(kprint), kprint->chars, kprint->len, &dummy)) {
        objtable_set_kv(gc, globals, obj_string_hash(kprint), kprint->chars, kprint->len,
                        obj_native_closure_val(
                            obj_native_closure_new_permanent(gc, native_print, "print", 0)));
        /* typeof / assembly 内建由 librefl 注册（System.Reflection 模块） */
    }
    /* System.Builtin：编译器开洞，注入底层内置命名空间（仅标准库可调） */
    install_builtin_module(gc, globals);
}

/* 创建 internal 标记的底层 native（System.Builtin 用）。 */
static ObjNativeClosure *make_builtin_native(Gc *gc, mica_native_fn fn, const char *name,
                                             uint32_t arity) {
    ObjNativeClosure *n = obj_native_closure_new_permanent(gc, fn, name, arity);
    n->internal = true;
    return n;
}

/* 注入 System.Builtin 命名空间（不占用任何模块文件）：
   globals["System"]["Builtin"] = { print, read_file, write_file, append_file, exists, read_lines }。
   这些 native 标记 internal=true，VM 调用时校验调用者必须是 System.* 标准库模块。 */
void install_builtin_module(Gc *gc, ObjTable *globals) {
    ObjString *ksys = obj_string_new_permanent(gc, "System", 6);
    Value sys_v = val_null();
    ObjMap *sys = NULL;
    if (objtable_get_kv(globals, obj_string_hash(ksys), ksys->chars, ksys->len, &sys_v) && val_is_obj(sys_v) &&
        val_as_obj(sys_v)->type == OBJ_MAP) {
        sys = (ObjMap *)val_as_obj(sys_v);
    } else {
        sys = obj_map_new_permanent(gc);
        objtable_set_kv(gc, globals, obj_string_hash(ksys), ksys->chars, ksys->len, obj_map_val(sys));
    }
    ObjMap *b = obj_map_new_permanent(gc);
    obj_map_set(gc, b, obj_string_val(obj_string_new_permanent(gc, "print", 5)),
                obj_native_closure_val(make_builtin_native(gc, native_print, "print", 0)));
    obj_map_set(gc, b, obj_string_val(obj_string_new_permanent(gc, "read_file", 9)),
                obj_native_closure_val(make_builtin_native(gc, native_read_file, "read_file", 1)));
    obj_map_set(gc, b, obj_string_val(obj_string_new_permanent(gc, "write_file", 10)),
                obj_native_closure_val(make_builtin_native(gc, native_write_file, "write_file", 2)));
    obj_map_set(gc, b, obj_string_val(obj_string_new_permanent(gc, "append_file", 11)),
                obj_native_closure_val(make_builtin_native(gc, native_append_file, "append_file", 2)));
    obj_map_set(gc, b, obj_string_val(obj_string_new_permanent(gc, "exists", 6)),
                obj_native_closure_val(make_builtin_native(gc, native_file_exists, "exists", 1)));
    obj_map_set(gc, b, obj_string_val(obj_string_new_permanent(gc, "read_lines", 10)),
                obj_native_closure_val(make_builtin_native(gc, native_read_lines, "read_lines", 1)));
    obj_map_set(gc, sys, obj_string_val(obj_string_new_permanent(gc, "Builtin", 7)), obj_map_val(b));
}

/* WeakRef 类 native 构造器：`WeakRef(obj)` → ObjWeakRef（弱引用，target 不阻止回收）。 */
static uint32_t native_weakref_ctor(mica_context *ctx, mica_value *args_mv, uint32_t argc, mica_value *res_mv) {
    Value *args = (Value *)args_mv; Value *res = (Value *)res_mv;
    res[0] = obj_weakref_val(obj_weakref_new(ctx->gc, argc > 0 ? args[0] : val_null()));
    return 1;
}

/* 创建 WeakRef 类对象（permanent；native_ctor 让 `WeakRef(obj)` 返回 ObjWeakRef）。
   供 extern class WeakRef（System.Runtime）注入用。 */
ObjClass *mica_weakref_class(Gc *gc) {
    ObjClass *k = obj_class_new_permanent(gc, obj_string_new_permanent(gc, "WeakRef", 7));
    k->native_ctor = native_weakref_ctor;
    return k;
}
