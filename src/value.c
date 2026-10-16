#include "value.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "obj.h"
#include "proto.h"
#include "str.h"

/* ---------- 工厂/谓词/取值/真值：已在 value.h（标量）与 obj.h（string/array/map）内联；
   value_eq/value_hash/float_as_int/hash64/obj_string_eq 已在 obj.h 内联。
   value.c 保留 value_type_name/value_print。 */

/* 自定义类型语义钩子（见 obj.h）：由上层模块（反射）注入。前置声明以便
   value_type_name / value_print 使用；未注入时退化为安全默认。 */
static MicaCustomHooks s_hooks;

/* ---------- 类型名与打印 ---------- */

const char *value_type_name(Value v) {
    if (val_is_float(v)) return "number";
    switch (val_tag(v)) {
        case MICA_TNULL: return "null";
        case MICA_TFALSE: case MICA_TTRUE: return "bool";
        case MICA_TSTR:
        case MICA_TARRAY:
        case MICA_TMAP:
        case MICA_TINSTANCE:
        case MICA_TCLOSURE:
        case MICA_TOBJ: {
            const Obj *o = val_as_obj(v);
            switch (o->type) {
                case OBJ_STRING: return "string";
                case OBJ_ARRAY: return "array";
                case OBJ_MAP: return "map";
                case OBJ_NATIVE_CLOSURE:
                case OBJ_CLOSURE:
                case OBJ_BUILTIN_METHOD: return "function";
                case OBJ_ERROR: return "error";
                case OBJ_ENUM_VAL: return "enum";
                case OBJ_CLASS: case OBJ_NATIVE_CLASS: return "class";
                case OBJ_INSTANCE: case OBJ_NATIVE_INSTANCE: return "object";
                case OBJ_CONTEXT:
                    return obj_context_is_coroutine((const mica_context *)o) ? "coroutine" : "context";
                case OBJ_WEAKREF: return "WeakRef";
                default:
                    /* 自定义类型（>= OBJ_TYPE_COUNT，如反射对象）：由注入钩子给出类型名 */
                    if (o->type >= OBJ_TYPE_COUNT && s_hooks.kind) return s_hooks.kind(o);
                    return "object";
            }
        }
        default: return "unknown";
    }
}

static void print_string_raw(FILE *out, const char *s, size_t len) {
    fwrite(s, 1, len, out);
}

typedef struct {
    FILE *out;
    bool first;
} MapPrintState;

static void print_map_entry(ObjString *key, Value value, void *user) {
    MapPrintState *st = (MapPrintState *)user;
    if (!st->first) fputs(", ", st->out);
    st->first = false;
    fputc('"', st->out);
    fprintf(st->out, "%.*s", (int)key->len, key->chars);
    fputc('"', st->out);
    fputs(": ", st->out);
    value_print(st->out, value);
}

/* 数字 → 文本（**统一**的数值格式化，value_print / value_to_cstr / 字符串拼接共用）：
   - 整数值（d == floor(d)，含 ±0）→ "%.0f"：**精确保留整数**。旧实现走 "%g"（6 位有效
     数字），于是 1000000 显示成 1e+06、9007199254740993 显示成 9.0072e+15 —— 对一门
     只有 double 的语言，大整数/ID/时间戳/JSON 往返全部静默失真。
   - 非整数 → "%.17g"：double 的最短往返精度（Lua 用 LUAI_NUMFFORMAT="%.14g" 偏短，
     这里取全精度，宁可长一点不失真）。
   写进调用方的栈缓冲（≥ 32 字节）并返回长度（不含 NUL）。 */
size_t value_num_to_buf(double d, char *buf, size_t cap) {
    if (d == 0.0 && signbit(d)) { /* -0.0：%.0f 给 "-0"，保留符号（与旧 %g 一致） */
        if (cap >= 3) { buf[0] = '-'; buf[1] = '0'; buf[2] = '\0'; }
        return 2;
    }
    if (isfinite(d) && d == floor(d)) {
        const int n = snprintf(buf, cap, "%.0f", d);
        return (n > 0 && (size_t)n < cap) ? (size_t)n : 0;
    }
    const int n = snprintf(buf, cap, "%.17g", d);
    return (n > 0 && (size_t)n < cap) ? (size_t)n : 0;
}

void value_print(FILE *out, Value v) {
    if (val_is_float(v)) {
        char buf[48];
        const size_t n = value_num_to_buf(val_as_float(v), buf, sizeof buf);
        if (n) fwrite(buf, 1, n, out);
        else fprintf(out, "%.17g", val_as_float(v));
        return;
    }
    switch (val_tag(v)) {
        case MICA_TNULL: fputs("null", out); break;
        case MICA_TFALSE: case MICA_TTRUE: fputs(val_as_bool(v) ? "true" : "false", out); break;
        case MICA_TSTR:
        case MICA_TARRAY:
        case MICA_TMAP:
        case MICA_TINSTANCE:
        case MICA_TCLOSURE:
        case MICA_TOBJ: {
            Obj *o = val_as_obj(v);
            switch (o->type) {
                case OBJ_STRING:
                    print_string_raw(out, ((ObjString *)o)->chars, ((ObjString *)o)->len);
                    break;
                case OBJ_ARRAY: {
                    const ObjArray *a = (const ObjArray *)o;
                    putc('[', out);
                    for (size_t i = 0; i < a->count; i++) {
                        if (i) fputs(", ", out);
                        value_print(out, a->items[i]);
                    }
                    putc(']', out);
                    break;
                }
                case OBJ_MAP: {
                    const ObjMap *m = (const ObjMap *)o;
                    MapPrintState st = {out, true};
                    putc('{', out);
                    obj_map_iter(m, print_map_entry, &st);
                    putc('}', out);
                    break;
                }
                case OBJ_NATIVE_CLOSURE: {
                    const ObjNativeClosure *n = (const ObjNativeClosure *)o;
                    fprintf(out, "<%s>", n->name ? n->name->chars : "native");
                    break;
                }
                case OBJ_CLOSURE: {
                    const ObjClosure *cl = (const ObjClosure *)o;
                    fprintf(out, "<%s>", cl->proto->name ? cl->proto->name->chars : "lambda");
                    break;
                }
                case OBJ_CAPVAL:
                    fputs("<capval>", out);
                    break;
                case OBJ_CLASS: {
                    const ObjClass *k = (const ObjClass *)o;
                    fprintf(out, "<%s>", k->name ? k->name->chars : "class");
                    break;
                }
                case OBJ_NATIVE_CLASS: {
                    const ObjNativeClass *k = (const ObjNativeClass *)o;
                    fprintf(out, "<%s>", k->name ? k->name->chars : "native-class");
                    break;
                }
                case OBJ_INSTANCE: {
                    const ObjInstance *i = (const ObjInstance *)o;
                    fprintf(out, "<%s>", i->klass->name ? i->klass->name->chars : "object");
                    break;
                }
                case OBJ_NATIVE_INSTANCE: {
                    const ObjNativeInstance *i = (const ObjNativeInstance *)o;
                    fprintf(out, "<%s>", i->klass->name ? i->klass->name->chars : "native-object");
                    break;
                }
                case OBJ_CONTEXT:
                    fputs(obj_context_is_coroutine((const mica_context *)o) ? "<coroutine>" : "<context>", out);
                    break;
                case OBJ_ERROR:
                    print_string_raw(out, ((ObjError *)o)->message->chars,
                                     ((ObjError *)o)->message->len);
                    break;
                case OBJ_ENUM_VAL: {
                    const ObjEnumVal *e = (const ObjEnumVal *)o;
                    if (e->set_name && e->case_name)
                        fprintf(out, "<%s.%s>", e->set_name->chars, e->case_name->chars);
                    else
                        fputs("<enum>", out);
                    break;
                }
                case OBJ_WEAKREF:
                    fputs("<WeakRef>", out);
                    break;
                case OBJ_TABLE:
                    fputs("<table>", out);
                    break;
                default:
                    /* 自定义类型（>= OBJ_TYPE_COUNT，如反射对象）：由注入钩子描述 */
                    if (o->type >= OBJ_TYPE_COUNT && s_hooks.kind) {
                        const char *k = s_hooks.kind(o);
                        const char *n = s_hooks.name ? s_hooks.name(o) : NULL;
                        fprintf(out, "<%s %s>", k ? k : "obj", n ? n : "?");
                    } else {
                        fputs("<obj>", out);
                    }
                    break;
            }
            break;
        }
    }
}

/* ---------- 自定义类型语义钩子（obj.h value_eq/value_hash 对 type >= OBJ_TYPE_COUNT 调用，
   以及 value_print 的打印分支）：由上层模块（反射）注入 —— 基础值系统不依赖上层模块；
   未注入时退化为引用相等 / 指针哈希 / "<obj>"（对任意自定义类型都安全）。 ---------- */

void value_set_custom_hooks(const MicaCustomHooks *hooks) {
    s_hooks = hooks ? *hooks : (MicaCustomHooks){0};
}

bool value_eq_custom(const Obj *a, const Obj *b) {
    if (s_hooks.eq) return s_hooks.eq(a, b);
    return a == b; /* 未注入：引用相等 */
}

uint64_t value_hash_custom(const Obj *o) {
    if (s_hooks.hash) return s_hooks.hash(o);
    return hash64((uint64_t)(uintptr_t)o); /* 未注入：指针哈希 */
}
