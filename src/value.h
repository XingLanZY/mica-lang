#ifndef MICA_VALUE_H
#define MICA_VALUE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "mica.h" /* mica_value（宿主 native 签名；val_wrap/val_unwrap 需其类型定义） */

/* ============================================================
   编译器抽象：强制内联/禁止内联与分支预测提示（MSVC / GCC / Clang 三编译器）。
   - MICA_FORCEINLINE：热函数强制内联。MSVC __forceinline；GCC/Clang always_inline。
   - MICA_NOINLINE：冷函数禁止内联（慢路径防膨胀）。
   - MICA_LIKELY/MICA_UNLIKELY：分支预测提示。
   ============================================================ */
#if defined(_MSC_VER)
#define MICA_FORCEINLINE static __forceinline
#define MICA_NOINLINE __declspec(noinline)
#define MICA_LIKELY(x) (x)
#define MICA_UNLIKELY(x) (x)
#elif defined(__GNUC__) || defined(__clang__)
#define MICA_FORCEINLINE static inline __attribute__((always_inline))
#define MICA_NOINLINE __attribute__((noinline))
#define MICA_LIKELY(x) __builtin_expect(!!(x), 1)
#define MICA_UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
#define MICA_FORCEINLINE static inline
#define MICA_NOINLINE
#define MICA_LIKELY(x) (x)
#define MICA_UNLIKELY(x) (x)
#endif

/* ============================================================
   Value = 8 字节 NaN-boxing（LuaJIT lj_obj.h 风格，2026-08-09）。

   布局（LJ_GC64 同款）：
     - number：double 位模式直接存。判定用 itype（算术右移 47 位取 32 位）：
         itype = (uint32_t)((int64_t)v >> 47)
       number（含正/负、Inf、**正负 NaN**）的 itype < MICA_TISNUM（0xFFFFFFF6）。
     - boxed（特殊 NaN）：v = (payload & MICA_GCVMASK) | ((uint64_t)tag << 47)，
       tag 是 32 位负补码（~N）。高 13 位全 1（0xFFF8...）→ itype = tag（0xFFFFFFFx）。
       低 47 位 payload = 指针 / 原始类型全 1（NULL）/ bool 无 payload。

     tag 分配（负补码，降序连续，BIGINT 最小）：
       MICA_TNULL      ~0u  = 0xFFFFFFFF   （it64 = -1，全 1）
       MICA_TFALSE    ~1u  = 0xFFFFFFFE
       MICA_TTRUE     ~2u  = 0xFFFFFFFD
       MICA_TSTR      ~3u  = 0xFFFFFFFC
       MICA_TARRAY    ~4u  = 0xFFFFFFFB
       MICA_TMAP      ~5u  = 0xFFFFFFFA
       MICA_TINSTANCE ~6u  = 0xFFFFFFF9
       MICA_TCLOSURE  ~7u  = 0xFFFFFFF8
       MICA_TOBJ      ~8u  = 0xFFFFFFF7   （最低的 boxed tag）
       MICA_TISNUM    ~9u  = 0xFFFFFFF6   （number 边界；< 为 number）

     ⚠️ TISNUM 必须**恰好**比最低 boxed tag（TOBJ = ~8u）大 1 个值（LuaJIT 同款规则，
     lj_obj.h:274-284 的 LJ_TISNUM = LJ_TLIGHTUD-1）。历史上写成 `~16u` = 0xFFFFFFEF，
     比注释里的 0xFFFFFFF0 小 1 → **负 NaN**（0xFFF8000000000000，itype 恒 0xFFFFFFF0）
     判成非 number 且落进对象区间（payload = 0 → 空指针）→ 任何让 NaN 存活的分配
     （register/map/instance/global + minor GC）都崩溃。~9u 后：
       正 NaN itype 0x0000FFF0、负 NaN 0xFFFFFFF0 → 均 < 0xFFFFFFF6 → number ✓
       最低对象 tag 0xFFFFFFF7 ≥ 0xFFFFFFF6 → boxed ✓

   判断全用宏（LuaJIT 风格）：一次 itype 算术右移 + 32 位单比较，热路径零解引用。
   对象区间 [TSTR, TOBJ] 用 LuaJIT 减法技巧单比较：
     tvisobj(v) = (itype - TISGCV) > (TISNUM - TISGCV)，TISGCV = TSTR+1。

   只保留 float 一种算术类型（2026-08-09）：所有整数用 number（double）表示，
   无独立 int/bigint 编码。
   ============================================================ */

typedef uint64_t Value;

typedef struct Obj Obj;

#define MICA_GCVMASK (((uint64_t)1 << 47) - 1)

/* ---- LuaJIT 风格 tag（负补码 32 位） ---- */
#define MICA_TNULL       (~0u)
#define MICA_TFALSE     (~1u)
#define MICA_TTRUE      (~2u)
#define MICA_TSTR       (~3u)
#define MICA_TARRAY     (~4u)
#define MICA_TMAP       (~5u)
#define MICA_TINSTANCE  (~6u)
#define MICA_TCLOSURE   (~7u)
#define MICA_TOBJ       (~8u)
/* number 边界：itype < 此值为 number。必须 = 最低 boxed tag（TOBJ = ~8u）+ 1，
   即 ~9u = 0xFFFFFFF6。写成 ~16u 会让负 NaN（itype 0xFFFFFFF0）漏进对象区间 → 崩溃。 */
#define MICA_TISNUM     (~9u)
#define MICA_TISGCV     (MICA_TSTR + 1u)

/* ---- 类型判断宏（热路径：一次算术右移 + 32 位比较） ---- */

#define MICA_ITYPE(v)      ((uint32_t)((int64_t)(v) >> 47))
/* 兼容别名（旧代码用 val_tag / val_payload） */
#define val_tag(v)         MICA_ITYPE(v)
#define val_payload(v)     ((v) & MICA_GCVMASK)
#define MICA_TVISNUM(v)    (MICA_ITYPE(v) < MICA_TISNUM)                      /* number 区 */
#define MICA_TVISNULL(v)    ((v) == (Value)UINT64_MAX)  /* null 全 1：全宽比较免 itype 移位 */
#define MICA_TVISBOOL(v)   ((MICA_ITYPE(v) == MICA_TFALSE) || (MICA_ITYPE(v) == MICA_TTRUE))
#define MICA_TVISSTR(v)    (MICA_ITYPE(v) == MICA_TSTR)
#define MICA_TVISARRAY(v)  (MICA_ITYPE(v) == MICA_TARRAY)
#define MICA_TVISMAP(v)    (MICA_ITYPE(v) == MICA_TMAP)
#define MICA_TVISINSTANCE(v) (MICA_ITYPE(v) == MICA_TINSTANCE)
#define MICA_TVISCLOSURE(v) (MICA_ITYPE(v) == MICA_TCLOSURE)
/* 对象区间 [TSTR, TOBJ]：LuaJIT 减法技巧，单无符号比较 */
#define MICA_TVISOBJ(v) \
  ((uint32_t)(MICA_ITYPE(v) - MICA_TISGCV) > (uint32_t)(MICA_TISNUM - MICA_TISGCV))

/* ---- 编码 / 取值（热路径内联） ---- */

/* boxed 编码：payload 低 47 位 + tag（负补码）高位移入 */
MICA_FORCEINLINE Value val_box(uint32_t tag, uint64_t payload) {
    return (payload & MICA_GCVMASK) | ((uint64_t)tag << 47);
}

MICA_FORCEINLINE Value val_null(void) { return (Value)UINT64_MAX; } /* it64=-1：全 1 */
/* bool 编码：MICA_TTRUE = ~2u = 0xFFFFFFFD、MICA_TFALSE = ~1u = 0xFFFFFFFE。
   val_true/val_false 是编译期常量直接物化（零分支）；val_bool(b) 用 `TFALSE - b`
   （b∈{0,1}，TTRUE = TFALSE-1）选 tag——免 test+branch/cmov。 */
MICA_FORCEINLINE Value val_true(void) { return val_box(MICA_TTRUE, 0); }
MICA_FORCEINLINE Value val_false(void) { return val_box(MICA_TFALSE, 0); }
MICA_FORCEINLINE Value val_bool(bool b) { return val_box(MICA_TFALSE - (uint32_t)b, 0); }
MICA_FORCEINLINE Value val_float(double f) { uint64_t b; memcpy(&b, &f, sizeof b); return b; }
/* 只保留 float 一种算术类型：整数一律用 number（double）表示。 */
MICA_FORCEINLINE Value val_int(int64_t i) {
    return val_float((double)i);
}
MICA_FORCEINLINE Value val_obj(Obj *o) { return val_box(MICA_TOBJ, (uintptr_t)o); }
/* 提升对象类型的带 tag 工厂（obj.h 的 obj_*_val 用）：tag 必须为 STRING..OBJ 负补码 */
MICA_FORCEINLINE Value val_obj_tag(uint32_t tag, const void *o) {
    return val_box(tag, (uintptr_t)o);
}

/* ---- 类型谓词（宏，见上）---- */
MICA_FORCEINLINE bool val_is_num(Value v) { return MICA_TVISNUM(v); }
MICA_FORCEINLINE bool val_is_float(Value v) { return MICA_TVISNUM(v); }
MICA_FORCEINLINE bool val_is_null(Value v) { return MICA_TVISNULL(v); }
MICA_FORCEINLINE bool val_is_bool(Value v) { return MICA_TVISBOOL(v); }
MICA_FORCEINLINE bool val_is_obj(Value v) { return MICA_TVISOBJ(v); }
MICA_FORCEINLINE bool val_is_string(Value v) { return MICA_TVISSTR(v); }
MICA_FORCEINLINE bool val_is_array(Value v) { return MICA_TVISARRAY(v); }
MICA_FORCEINLINE bool val_is_map(Value v) { return MICA_TVISMAP(v); }
MICA_FORCEINLINE bool val_is_instance(Value v) { return MICA_TVISINSTANCE(v); }
MICA_FORCEINLINE bool val_is_closure(Value v) { return MICA_TVISCLOSURE(v); }

/* 两值皆 number。封装 `val_is_num(a) && val_is_num(b)`——注意不能合并成单次位运算：
   number 区 `MICA_ITYPE < MICA_TISNUM` 非规则区间（boxed tag itype 0xFFFFFFF0..0xFFFFFFFF
   与负 double 高位 itype 重叠），对 OR/AND 不封闭。实测 `(ITYPE(a)|ITYPE(b)) < TISNUM`
   对合法数字对（有限负 double | 正 NaN，两者现都被 val_is_num 判为数字）产生假阴性。
   编译器对两次检查生成的代码已接近最优（算术快路径 2 cmpq+jae），此封装仅为可读性。 */
MICA_FORCEINLINE bool val_both_num(Value a, Value b) {
    return MICA_TVISNUM(a) && MICA_TVISNUM(b);
}

/* ---- 取值 ---- */

MICA_FORCEINLINE bool val_as_bool(Value v) { return MICA_ITYPE(v) == MICA_TTRUE; }
MICA_FORCEINLINE double val_as_float(Value v) { double f; memcpy(&f, &v, sizeof f); return f; }
/* 对象指针解箱（调用方保证 val_is_obj） */
MICA_FORCEINLINE Obj *val_as_obj(Value v) { return (Obj *)(uintptr_t)(v & MICA_GCVMASK); }

/* 严格布尔语义（2026-08-10）：仅 bool 类型有真值，非 bool 一律报错（由布尔上下文
   调用方 vm_truthy_type_error 处理）。val_truthy 只解释 bool——传入非 bool 是调用方错误。 */
MICA_FORCEINLINE bool val_truthy(Value v) {
    return MICA_TVISBOOL(v) && val_as_bool(v);
}

/* mica_value（宿主 native 签名）↔ 内部 Value 互转。mica_value 与 Value 同 8 字节同布局
   （NaN-boxing），memcpy 位拷贝。mica_native_fn 参数/结果用 mica_value，内部 VM 用 Value。 */
MICA_FORCEINLINE mica_value val_wrap(Value v) {
    mica_value mv;
    memcpy(mv.bits, &v, sizeof v);
    return mv;
}
MICA_FORCEINLINE Value val_unwrap(mica_value mv) {
    Value v;
    memcpy(&v, mv.bits, sizeof v);
    return v;
}

/* 相等（数值 double；字符串内容；array/map/func 引用）与哈希已移到 obj.h 内联
   （需 ObjString/ObjReflType，见 obj.h 的 value_eq/value_hash/obj_string_eq）。 */

/* 类型的简短名字符串（错误消息用；typeof() 现在返回 Type 对象，见 librefl.h） */
const char *value_type_name(Value v);

/* 打印：字符串输出原文（不带引号），数组 [a, b]，map 按稠密序 {k: v}，
   func/native 为 <name>。print 内建与测试用。 */
void value_print(FILE *out, Value v);

/* 数字 → 文本（统一的数值格式化；value_print / value_to_cstr / 字符串拼接共用）：
   整数值用 "%.0f"（**精确保留整数**，旧 %g 会把 1000000 打成 1e+06），非整数用 "%.17g"。
   写进调用方缓冲（≥ 48 字节），返回长度（不含 NUL）；失败返回 0。 */
size_t value_num_to_buf(double d, char *buf, size_t cap);

#endif /* MICA_VALUE_H */
