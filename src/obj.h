#ifndef MICA_OBJ_H
#define MICA_OBJ_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <assert.h>

#include "mica.h"
#include "value.h"

typedef struct String String; /* 值类型字符串（str.h 定义完整结构；此处仅前置声明供字段助手签名） */

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4200) /* C11 柔性数组成员（ObjString.chars[]） */
#endif

/* GC 托管对象：基结构 + 具体类型。所有对象经 Gc 分配：
   运行期对象走 gc_alloc（nursery/老年代，可被收集），
   编译期对象走 gc_alloc_permanent（OBJ_PERMANENT，GC 永不回收）。
   不变量：permanent 对象的字段不指向运行期对象（编译期构造保证）。 */

typedef struct Gc Gc;
typedef struct Proto Proto;
typedef struct CallFrame CallFrame; /* 调用帧（定义在 state.h，Lua CallInfo 风格） */
typedef struct ObjTable ObjTable;
typedef struct mica_context mica_context; /* 每个协程是一个 mica_context（见 state.h） */
typedef struct mica_runtime mica_runtime; /* 进程级全局状态（见 state.h） */

typedef enum {
    OBJ_STRING,
    OBJ_ARRAY,
    OBJ_MAP,
    OBJ_NATIVE_CLOSURE,   /* 宿主 native 闭包（Lua CClosure 风格：mica_native_fn + 捕获值） */
    OBJ_TABLE,
    OBJ_CAPVAL,
    OBJ_CLOSURE,
    OBJ_ERROR,
    OBJ_ENUM_VAL,         /* 枚举值（enum payload）：Color.Red(args) 构造 */
    OBJ_CLASS,
    OBJ_INSTANCE,
    OBJ_BUILTIN_METHOD,
    OBJ_WEAKREF,          /* 弱引用（WeakRef）：target 不阻止回收（见 gc.c gc_clear_weakrefs） */
    OBJ_CONTEXT,          /* mica_context（可 GC 的执行线程/协程，见 state.h） */
    OBJ_NATIVE_CLASS,     /* 原生类（绑定 C 结构体 userdata 的类，见 ObjNativeClass） */
    OBJ_NATIVE_INSTANCE,  /* 原生实例（包装 C 结构体 userdata，见 ObjNativeInstance） */
    /* 反射对象类型（Type/Method/Field/Property/Module/Assembly）移入 librefl（LREFL_* 自定义类型） */
    OBJ_TYPE_COUNT,
} ObjType;

/* flags 位 */
#define OBJ_PERMANENT   (1u << 0)   /* 编译期对象，GC 跳过 */
#define OBJ_HASHED      (1u << 1)   /* ObjString 懒哈希已缓存（obj_string_hash 访问器用） */
#define OBJ_CAPVAL_CLOSED (1u << 2) /* ObjCapVal 已关闭（p/value union 下用标志位判 open/closed） */

/* 可 GC 对象头（Lua 式 CommonHeader）：Obj 与所有子类结构体开头必须展开此宏。
   next 在最前（8 字节指针），随后 4 个单字节标志，共 12 字节；首个 8 对齐字段（如 gclist）
   落在 offset 16（4 字节填充）。 */
#define ObjHeader \
    Obj *next;          /* all_obj 链（所有运行期对象） / permanent 链 */ \
    uint8_t type; \
    uint8_t mark;       /* 双白 + 三色：bit0/1 = WHITE0/WHITE1（currentwhite 二选一）、bit2 = BLACK、0 = 灰 */ \
    uint8_t age;        /* 渐进代龄 6 态（见 gc.h MICA_AGE_*）：NEW/SURV/OLD0/OLD1/OLD/TOUCHED */ \
    uint8_t flags;

typedef struct Obj {
    ObjHeader
} Obj;

/* 统一链节点：gray/gray_again 链用 gclist 字段。**所有可能引用其它对象**的类型
   （array/map/table/class/closure/capval/error/enum/instance/coroutine/context/refl_*）都在
   ObjHeader 后紧跟 `Obj *gclist`，保证 OBJ_GCLIST(o) 偏移一致（offset 16）。
   叶子/无引用类型（string/native/builtin_method/coro_method/weakref）不入 gray。 */
typedef struct GcGCNode {
    ObjHeader
    Obj *gclist;
} GcGCNode;

#define OBJ_GCLIST(o) (((GcGCNode *)(o))->gclist)

typedef struct {
    ObjHeader
    uint32_t len;
    uint32_t hash;       /* 懒哈希缓存（obj_string_hash 首次访问计算；OBJ_HASHED 位标记） */
    char chars[];        /* NUL 结尾 */
} ObjString;

typedef struct {
    ObjHeader           /* 12 字节 + 4 填充（offset 12-15，ObjHeader 注释） */
    uint8_t elements_kind; /* 运行时元素类型提示（V8 elements-kind 风格，2026-08-13 C1）：
                              ARR_ELEM_EMPTY=未写 / NUM=只写过 number / BOOL=只写过 bool /
                              ANY=混合或其它（保守）。GC 据此跳过 NUM/BOOL 数组的元素扫描；
                              ANY 恒保守。非指针，无需 GC trace。
                              放在 gclist 前利用 ObjHeader 尾部 4 字节填充（offset 12），
                              ObjArray 省 8 字节（56→48）。 */
    Obj *gclist;           /* gray/gray_again 链（容器统一偏移 16，见 GcGCNode） */
    Value *items;
    uint32_t count;      /* 元素数（< 4G；收窄自 size_t：数组结构 48→40 字节） */
    uint32_t capacity;   /* 容量（Value 数） */
} ObjArray;

/* 数组元素类型（运行时维护，GC 扫描优化提示；错误时退化为 ANY，语义不变） */
enum {
    ARR_ELEM_EMPTY = 0,
    ARR_ELEM_NUM = 1,
    ARR_ELEM_BOOL = 2,
    ARR_ELEM_ANY = 3,
};

/* 数组写入后维护 elements_kind（obj_array_set/push 内部调用；解释器快路径直接写 items
   处也须调用——见 vm.c INDEX_SET/SETI 快路径）。非写入（数组读）不改变 kind。 */
void obj_array_update_kind(ObjArray *a, Value v);

/* 用户 map 值：键为 GC 托管的 ObjString*（语言层约束字符串键）。
   结构 = 稀疏桶 + 稠密条目数组（键值复合在一个 ObjMapEntry）：
   - sparse[]：哈希桶 → 稠密下标（MAP_NULL_ELEM=空、MAP_TOMB=墓碑）。
   - entries[]：插入序稠密数组，每条 = {ObjString *key, Value value}（key 是 GC 对象，
     map 持有引用，map_trace 须同时 trace 键与值；key 不 str_free，由 GC 回收）。
   键是 GC 对象：map_trace 既 trace 键也 trace 值。 */
typedef struct ObjMapEntry {
    ObjString *key;       /* GC 键（map 持有引用） */
    Value value;
} ObjMapEntry;

typedef struct ObjMap {
    ObjHeader
    Obj *gclist;           /* gray/gray_again 链（容器统一偏移） */
    size_t *sparse;      /* 哈希桶 → 稠密下标；NullElement/Tombstone 哨兵 */
    uint32_t sparse_cap; /* 桶数（< 4G；收窄自 size_t：map 结构 72→56 字节） */
    uint32_t sparse_used;/* 存活 + 墓碑 */
    ObjMapEntry *entries; /* 稠密条目数组（插入序；key+value 复合） */
    uint32_t count;      /* 存活条目数 */
    uint32_t cap;        /* entries 容量 */
} ObjMap;

/* 字符串/数组/Map/实例/闭包谓词已移入 value.h（2026-08-08 提升为独立 ValueType 后
   只需 Value.type，无需 Obj.type）。 */

/* 内置类型成员方法 / 原生构造器统一使用**公开 API 类型** mica_native_fn（定义于 mica.h）：
   `uint32_t (*)(mica_context *ctx, mica_value *args, uint32_t argc, mica_value *results)`。
   —— 不再有内部专用的 mica_native_fn：宿主 native、内建方法、反射方法三者**同一签名**，
   实现"一种回调类型贯穿全栈"。mica_value 与内部 Value 同布局（8 字节 uint64），
   内部实现在函数体开头 `Value *a = (Value *)args;` 强转即可，零开销。
   注：旧 NativeFn 的 `void *userdata` 参数已删除——全项目从未使用（均为 `(void)userdata;`）。 */

/* 原生闭包（Lua CClosure 风格，替代旧 ObjNative）：函数体是宿主 native（mica_native_fn，
   公开 API 签名，参数/结果为 mica_value），可携带 n_capvals 个捕获值（Value capvals[]）。
   - 公开注册（mica_register_native/function/method/property）直接存 mica_native_fn。
   - 内部内置（builtin.c System.Builtin / librefl.c）也统一为 mica_native_fn。
   - 与 ObjClosure 对齐：n_capvals + gclist 布局（gclist offset 16），capvals 需 GC trace。
   - name：打印/错误消息（value_print <name>）；arity：缺参补 null；internal：System.Builtin
     仅标准库可调守卫。 */
typedef struct ObjNativeClosure {
    ObjHeader
    uint8_t n_capvals;
    Obj *gclist;           /* gray/gray_again 链（容器统一偏移，capvals 引用对象须 trace） */
    mica_native_fn func;   /* 宿主 native（mica_value* 签名） */
    ObjString *name;       /* permanent：打印/错误用 */
    uint32_t arity;
    bool internal;         /* 内部内置（System.Builtin）：仅标准库（System.* 模块）可调用 */
    bool is_method;        /* 方法 native：receiver@slot1、实参从 slot2、arity 含 receiver；
                              普通 native：slot1=null self、实参从 slot2、arity=用户参数数 */
    Value capvals[];       /* 柔性数组：n_capvals 个捕获值（GC trace；无则空） */
} ObjNativeClosure;

/* 开放捕获值（open capval，Lua 式懒装箱）：被 lambda 捕获的外层变量的共享盒子。
   - open：p 指向当前帧的栈槽，直接读写栈槽（声明函数读/写走普通寄存器，天然共享）。
   - 宿主帧返回/协程终结时 close：把 *p 拷入 value、p 改指向 &value，与栈解耦。
   - p/value 共用 union（同址）：open 时用 p（栈槽指针），closed 时用 value（自有值）。
     union 下 `cap->p == &cap->value` 恒真（同址），判 open/closed 须用
     OBJ_CAPVAL_CLOSED 标志位（obj.c/vm.c 统一）。
   - 栈 realloc 时 open 捕获值的 p 会由 ensure_stack 按偏移修正。
   - open_next/open_prev：双向链表，open 状态时挂在所属 context 的 opencapval 链上
     （Lua 的 lua_State.openupval 等价）；close 时摘除。 */
typedef struct ObjCapVal {
    ObjHeader
    struct ObjCapVal *open_next;  /* open 链（context.opencapval），勿用 next（与 ObjHeader 冲突） */
    struct ObjCapVal *open_prev;  /* 双向链：摘除 O(1) */
    union {
        Value *p;      /* open: 指向栈槽；closed: 指向 &value（与 value 同址） */
        Value value;   /* closed 时持有值（open 时无效；gc_alloc 零初始化为 null） */
    };
} ObjCapVal;

/* 闭包：Mica 的唯一函数值类型。
   - 普通函数/方法/顶层函数：编译期创建 permanent 闭包（capvals 空）。
   - lambda：编译期 Proto 入常量池（permanent closure），运行时 OP_CLOSURE 创建闭包（填捕获值）。
   - gclist 字段对齐 Lua 5.0 ClosureHeader（CommonHeader; nupvalues; gclist）：Mica 用作
     gray/gray_again 链节点（容器统一偏移，见 GcGCNode）。permanent 闭包恒 NULL（不参与收集）。 */
typedef struct ObjClosure {
    ObjHeader
    uint8_t n_capvals;
    Obj *gclist;           /* gray/gray_again 链（容器统一偏移） */
    Proto *proto;                  /* 函数体（permanent；name/arity/max_regs/is_throws 等元数据） */
    ObjCapVal *capvals[];        /* 柔性数组：n_capvals 个捕获值 */
} ObjClosure;

/* 错误值（Zig/Swift 风格：错误是返回值）。
   匿名 `error("msg")`：set_name/case_name 为 NULL，无 payload。
   具名 `Set.Case(args...)`：携带 set/case 名与载荷值 + 载荷标识符。 */
typedef struct {
    ObjHeader
    uint32_t n_payload;        /* @12：利用 ObjHeader 尾部 4 字节填充（gclist 须在 offset 16）
                                  载荷个数；收窄自 size_t 尾部（省 8 字节/错误值） */
    Obj *gclist;               /* gray/gray_again 链（统一偏移） */
    ObjString *message;        /* .message / 字符串拼接 */
    ObjString *set_name;       /* NULL = 匿名 */
    ObjString *case_name;      /* NULL = 匿名 */
    ObjString **payload_names; /* 载荷标识符（case 声明的顺序）；匿名 NULL */
    Value *payload;            /* 载荷值；匿名 NULL */
} ObjError;

/* 枚举值（enum payload）：`Color.Red(args...)` 构造的值对象。
   set_name/case_name 为 permanent（编译期常量池）；payload 为运行期载荷值数组。 */
typedef struct {
    ObjHeader
    uint32_t n_payload;    /* @12：利用 ObjHeader 尾部填充（gclist 须在 offset 16）；
                              载荷个数，收窄自 size_t 尾部（省 8 字节/枚举值） */
    Obj *gclist;               /* gray/gray_again 链（统一偏移） */
    ObjString *set_name;   /* 枚举名（permanent） */
    ObjString *case_name;  /* case 名（permanent） */
    Value *payload;        /* 载荷值数组（malloc，finalize 释放） */
} ObjEnumVal;

/* 类（编译期 permanent 对象）：名字、父类、实例/静态方法、属性 getter/setter、
   字段名、字段默认值方法。 */
typedef struct ObjClass {
    ObjHeader
    Obj *gclist;               /* gray/gray_again 链（容器统一偏移） */
    Gc *gc;                  /* 创建时的 Gc（permanent，用于构造查找键） */
    ObjString *name;           /* permanent */
    ObjString *module;         /* permanent：所属模块名（System.IO 等），非模块类 NULL */
    struct ObjClass *superclass;   /* NULL = 无父类 */
    ObjTable *methods;         /* permanent：实例方法 name → ObjClosure */
    ObjTable *static_methods;  /* permanent：静态方法 name → ObjClosure */
    ObjTable *getters;         /* permanent：属性 getter name → ObjClosure */
    ObjTable *setters;         /* permanent：属性 setter name → ObjClosure */
    bool has_getters;          /* 本类或父类是否声明过 getter（无则 VM 跳过 getter 查找） */
    bool has_setters;          /* 本类或父类是否声明过 setter（无则 VM 跳过 setter 查找） */
    ObjString **field_names;   /* 本类声明的字段名（不含父类） */
    uint32_t n_field_names;    /* 收窄自 size_t（字段数 < 4G） */
    ObjTable *field_index;     /* permanent：字段名(String) → int 绝对下标（继承前缀布局，含全部父类字段） */
    uint32_t n_total_fields;   /* 实例字段总数（含继承）＝ ObjInstance.fields[] 长度 */
    uint32_t last_field_hash;  /* 单槽字段索引缓存（P6）：最近查询字段名哈希 + chars 指针
                                  + 绝对下标。命中（hash + chars 指针相等）免 field_index
                                  哈希表查找（热循环同 class+同字段重复访问）。chars 指针
                                  比较即内容相等（编译期字段名 String 不可变，同名字恒同指针）。 */
    const char *last_field_chars;
    int32_t last_field_idx;    /* -1 = 未声明字段；配合 field_cache_valid 使用 */
    bool field_cache_valid;
    ObjClosure *defaults;      /* 字段默认值闭包（self 参数），NULL = 无 */
    mica_native_fn native_ctor;      /* 可选：非 NULL 时 X(args) 调它返回特殊对象（如 WeakRef），跳过默认实例构造 */
} ObjClass;

/* 实例：所属类 + 柔性数组字段（声明字段编译期固定，长度 klass->n_total_fields）。
   未设字段 = val_null；未声明字段访问/赋值 → 运行期报错（无动态字段）。 */
typedef struct ObjInstance {
    ObjHeader
    Obj *gclist;               /* gray/gray_again 链（统一偏移） */
    ObjClass *klass;
    Value fields[];            /* 柔性数组：声明字段值，长度 klass->n_total_fields */
} ObjInstance;

/* 原生类（绑定 C 结构体 userdata，QuickJS JSClassDef 风格）——独立字段，**不继承 ObjClass**：
   普通类（ObjClass）实例是 Mica 声明字段（Value[]）；原生类实例是宿主 C 结构体
   （opaque userdata 字节）。原生类是 permanent 描述符，自带方法/静态方法/属性表。
   无 gclist（permanent 不入 gray 链；子对象方法表都是 permanent）。 */
typedef struct ObjNativeClass {
    ObjHeader
    ObjString *name;              /* permanent */
    ObjString *module;            /* permanent（可选） */
    uint32_t userdata_size;       /* ObjNativeInstance.userdata[] 字节数（< 4G） */
    mica_native_finalizer finalizer; /* 实例回收回调（可为 NULL） */
    ObjTable *methods;            /* permanent：实例方法 name → ObjNativeClosure/ObjClosure */
    ObjTable *static_methods;     /* permanent：静态方法 name → ... */
    ObjTable *getters;            /* permanent：属性 getter name → ... */
    ObjTable *setters;            /* permanent：属性 setter name → ... */
    bool has_getters;
    bool has_setters;
} ObjNativeClass;

/* 原生实例：包装宿主 C 结构体（opaque userdata[]，长度 klass->userdata_size）。
   - userdata 是原始字节，GC **无法追踪**（C 结构体，非 GC 对象）→ 无 gclist、非 grayable；
     实例回收时只能经 klass->finalizer(userdata) 释放 C 资源。
   - klass 是 permanent（永不回收），故实例唯一引用无须 trace。 */
typedef struct ObjNativeInstance {
    ObjHeader
    ObjNativeClass *klass;
    uint8_t userdata[];       /* klass->userdata_size 字节 */
} ObjNativeInstance;

/* ---------- 执行状态 ---------- */

#define MICA_DEFER_STACK 64      /* 每 context defer 栈初始容量（变长，可增长） */
#define MICA_CORO_STACK_INIT 128 /* 协程 context 栈初始大小（Value 数，1KB；ensure_stack 弹性增长） */

/* 执行状态并入 mica_context（见 state.h）——每个协程是一个 context。
   协程状态常量（mica_context.coro_status）。 */
#define CORO_SUSPENDED 0
#define CORO_RUNNING   1
#define CORO_NORMAL    2
#define CORO_DEAD      3
#define CORO_NONE      4   /* 主 context（非协程）：status 低 4 位用 */

/* 内置类型成员方法（数组/字符串/Map 的 push/pop/replace 等）：native + 调用时 receiver 在 slot1。 */
typedef struct {
    ObjHeader
    mica_native_fn fn;
} ObjBuiltinMethod;

/* 弱引用（WeakRef）：target 是弱引用（GC 不因它保持 target 存活）。
   next_weakref 挂入 Gc 的 weakrefs 链表；收集的"标记完成、清扫前"钩子（gc_clear_weakrefs）
   把目标已死的 weakref.target 置 null。 */
typedef struct ObjWeakRef {
    ObjHeader
    Value target;          /* 弱引用目标；已回收则为 null */
    struct ObjWeakRef *next_weakref; /* GC 弱引用注册表链表 */
} ObjWeakRef;

/* ============================================================
   字符串哈希/相等 与 值相等/哈希（热路径内联：成员访问的表探测、VM 比较指令）。
   定义在 obj.h 而非 value.h——需 ObjString；obj.h 已含 value.h。
   不变量：value_eq(a,b) 为真 ⇒ value_hash(a) == value_hash(b)。
   自定义类型（>= OBJ_TYPE_COUNT，如反射 LREFL_*）经 value_eq_custom/value_hash_custom 处理。
   ============================================================ */

/* 懒哈希计算（obj.c 实现）：首次访问时 fnv_hash 计算并缓存（OBJ_HASHED 位）。
   obj_string_new/obj_string_new_permanent 不预计算——只在哈希真正被使用（map/表键、
   相等比较、value_hash）时才计算，避免每次构造字符串付 O(len) fnv_hash。 */
uint32_t obj_string_hash_compute(ObjString *s);

MICA_FORCEINLINE uint32_t obj_string_hash(const ObjString *s) {
    ObjString *m = (ObjString *)s; /* 缓存写：单线程，cast 掉 const 写 hash 字段 */
    if (MICA_UNLIKELY(!(m->flags & OBJ_HASHED))) return obj_string_hash_compute(m);
    return m->hash;
}

MICA_FORCEINLINE bool obj_string_eq(const ObjString *a, const ObjString *b) {
    if (a == b) return true;
    if (obj_string_hash(a) != obj_string_hash(b)) return false; /* 哈希不同必不等：快速拒绝 */
    if (a->len != b->len) return false;
    return memcmp(a->chars, b->chars, a->len) == 0;
}

/* ObjString vs 原始键（hash/chars/len）比较：标识符池（String）与 ObjString 键共用。
   String.hash == ObjString.hash == fnv_hash(...)（32 位 Murmur3），故哈希直接可比。 */
MICA_FORCEINLINE bool obj_string_eq_kv(const ObjString *a, uint32_t hash, const char *chars,
                                       size_t len) {
    return obj_string_hash(a) == hash && a->len == len && memcmp(a->chars, chars, len) == 0;
}

/* float 若为整数值且在 int64 范围内 → 按 int64 哈希，保证与 INT 键一致。 */
MICA_FORCEINLINE bool float_as_int(double f, int64_t *out) {
    if (!isfinite(f) || floor(f) != f) return false;
    if (f >= -9223372036854775808.0 && f < 9223372036854775808.0) {
        *out = (int64_t)f;
        return true;
    }
    return false;
}

MICA_FORCEINLINE uint64_t hash64(uint64_t x) {
    x ^= x >> 33;
    x *= UINT64_C(0xff51afd7ed558ccd);
    x ^= x >> 33;
    x *= UINT64_C(0xc4ceb9fe1a85ec53);
    x ^= x >> 33;
    return x;
}

/* 自定义类型（>= OBJ_TYPE_COUNT）的语义钩子：由上层模块（反射）注入。
   基础值系统因此不依赖任何上层模块；未注入的槽位退化为安全默认（引用相等 /
   指针哈希 / 打印 "<obj>"）。依赖倒置：下层定义接口，上层提供实现。
   注：钩子为进程级（语言运行时每进程一个），在 rt 初始化时注册一次。 */
typedef bool        (*MicaCustomEqFn)(const Obj *a, const Obj *b);
typedef uint64_t    (*MicaCustomHashFn)(const Obj *o);
typedef const char *(*MicaCustomKindFn)(const Obj *o); /* 类型标签，如 "Type" */
typedef const char *(*MicaCustomNameFn)(const Obj *o); /* 对象名，可返回 NULL */

typedef struct {
    MicaCustomEqFn   eq;    /* NULL → 引用相等 */
    MicaCustomHashFn hash;  /* NULL → 指针哈希 */
    MicaCustomKindFn kind;  /* NULL → 打印 "<obj>"；否则打印 "<kind name>" */
    MicaCustomNameFn name;  /* 与 kind 配套，可为 NULL（打印 "?"） */
} MicaCustomHooks;

void value_set_custom_hooks(const MicaCustomHooks *hooks);
bool value_eq_custom(const Obj *a, const Obj *b);
uint64_t value_hash_custom(const Obj *o);

/* 相等：number 直接 double 比较；字符串内容；自定义类型（Type 结构相等等）；其余引用相等。 */
MICA_FORCEINLINE bool value_eq(Value a, Value b) {
    if (MICA_TVISNUM(a)) {
        if (MICA_TVISNUM(b)) return val_as_float(a) == val_as_float(b);
        return false;
    }
    if (MICA_TVISNUM(b)) return false;
    /* 两者 boxed */
    const uint32_t ta = val_tag(a), tb = val_tag(b);
    if (ta != tb) return false;
    switch (ta) {
        case MICA_TNULL: return true;
        case MICA_TFALSE: case MICA_TTRUE: return val_payload(a) == val_payload(b);
        case MICA_TSTR:
        case MICA_TARRAY:
        case MICA_TMAP:
        case MICA_TINSTANCE:
        case MICA_TCLOSURE:
        case MICA_TOBJ: {
            Obj *oa = val_as_obj(a), *ob = val_as_obj(b);
            if (oa == ob) return true;
            if (oa->type == OBJ_STRING && ob->type == OBJ_STRING) {
                return obj_string_eq((const ObjString *)oa, (const ObjString *)ob);
            }
            if (oa->type >= OBJ_TYPE_COUNT && ob->type >= OBJ_TYPE_COUNT) {
                return value_eq_custom(oa, ob); /* 自定义类型（反射 LREFL_* 等）：librefl 实现 */
            }
            return false; /* array/map/func/native 引用相等 */
        }
        default: return false;
    }
}

MICA_FORCEINLINE uint64_t value_hash(Value v) {
    if (MICA_TVISNUM(v)) { /* number */
        int64_t i;
        if (float_as_int(val_as_float(v), &i)) return hash64((uint64_t)i);
        return hash64(v); /* double 位模式 */
    }
    const uint32_t t = val_tag(v);
    switch (t) {
        case MICA_TNULL: return UINT64_C(0x9e3779b97f4a7c15);
        case MICA_TFALSE: case MICA_TTRUE: return (t == MICA_TTRUE) ? 1 : 0;
        case MICA_TSTR:
        case MICA_TARRAY:
        case MICA_TMAP:
        case MICA_TINSTANCE:
        case MICA_TCLOSURE:
        case MICA_TOBJ: {
            Obj *o = val_as_obj(v);
            if (o->type == OBJ_STRING) {
                return obj_string_hash((const ObjString *)o);
            }
            if (o->type >= OBJ_TYPE_COUNT) {
                return value_hash_custom(o); /* 自定义类型（反射 LREFL_* 等）：librefl 实现 */
            }
            return hash64((uint64_t)(uintptr_t)o);
        }
        default: return 0;
    }
}

/* ---------- 工厂（Gc 由调用方持有；permanent 版本编译期用） ---------- */

ObjString *obj_string_new(Gc *gc, const char *s, size_t len);
ObjString *obj_string_new_permanent(Gc *gc, const char *s, size_t len);

ObjArray *obj_array_new(Gc *gc, size_t cap);
ObjArray *obj_array_new_permanent(Gc *gc, size_t cap);
void obj_array_set(Gc *gc, ObjArray *a, size_t i, Value v);   /* 越界断言；写屏障 */
void obj_array_push(Gc *gc, ObjArray *a, Value v);             /* 写屏障 */
Value obj_array_pop(Gc *gc, ObjArray *a);                      /* 空返回 null；写屏障(无) */
/* 数组读/长度：纯读（无写屏障），热路径（OP_INDEX/OP_LENGTH 快路径）内联，消除跨 TU 调用。 */
MICA_FORCEINLINE Value obj_array_get(const ObjArray *a, size_t i) {
    assert(i < a->count);
    return a->items[i];
}
MICA_FORCEINLINE size_t obj_array_count(const ObjArray *a) {
    return a->count;
}

ObjMap *obj_map_new(Gc *gc);
ObjMap *obj_map_new_permanent(Gc *gc);  /* permanent：GC 永不回收（宿主/内置命名空间用） */
/* 键强制为字符串（ObjString 值）。obj_map_set/get/delete 直接存/查 ObjString*（不拷贝）；
   obj_map_set_kv/get_kv/*_by_string 接受原始键（hash/chars/len），缺键时内部创建 ObjString。
   非字符串键由调用方（vm/builtin）先检查报错。 */
bool obj_map_set(Gc *gc, ObjMap *m, Value key, Value val);     /* 已存在返回 false；键值写屏障 */
bool obj_map_get(const ObjMap *m, Value key, Value *out);      /* 缺键返回 false */
bool obj_map_set_kv(Gc *gc, ObjMap *m, uint32_t hash, const char *chars, size_t len,
                    Value val); /* 字符串键插入/更新（原始键，缺键建 ObjString；写屏障） */
bool obj_map_get_kv(const ObjMap *m, uint32_t hash, const char *chars, size_t len,
                    Value *out); /* 字符串键快速查找（标识符池用） */
bool obj_map_get_by_string(const ObjMap *m, const char *key, size_t key_len, uint32_t hash,
                           Value *out); /* obj_map_get_kv 的别名（用户指定签名） */
bool obj_map_set_by_string(Gc *gc, ObjMap *m, const char *key, size_t key_len, uint32_t hash,
                           Value val); /* 字符串键插入/更新：缺键建 ObjString 存表，已有按内容更新 */
bool obj_map_delete(Gc *gc, ObjMap *m, Value key);
size_t obj_map_size(const ObjMap *m);

typedef void (*ObjMapIterCb)(ObjString *key, Value value, void *user);
void obj_map_iter(const ObjMap *m, ObjMapIterCb cb, void *user); /* 稠密序 */

ObjNativeClosure *obj_native_closure_new_permanent(Gc *gc, mica_native_fn fn, const char *name,
                                                   uint32_t arity);

/* 原生类 / 原生实例（绑定 C 结构体 userdata）。 */
ObjNativeClass *obj_native_class_new_permanent(Gc *gc, ObjString *name, ObjString *module,
                                               size_t userdata_size,
                                               mica_native_finalizer finalizer);
ObjNativeInstance *obj_native_instance_new(Gc *gc, ObjNativeClass *klass); /* userdata 零初始化 */
void *obj_native_instance_data(ObjNativeInstance *inst);                  /* userdata 指针 */
Value obj_native_class_val(ObjNativeClass *c);
Value obj_native_instance_val(ObjNativeInstance *i);
/* 沿继承链查原生类实例方法 / 静态方法 / 属性 getter/setter（原始键，vm get_member 用）。 */
ObjClosure *obj_native_class_find_method_kv(ObjNativeClass *klass, uint32_t hash, const char *chars,
                                            size_t len);
Value obj_native_class_find_static_kv(ObjNativeClass *klass, uint32_t hash, const char *chars,
                                      size_t len);
Value obj_native_class_find_getter_kv(ObjNativeClass *klass, uint32_t hash, const char *chars,
                                      size_t len);
Value obj_native_class_find_setter_kv(ObjNativeClass *klass, uint32_t hash, const char *chars,
                                      size_t len);

/* 捕获值（capvalue）：新建（closed 初始态，p=&value，值为 v；调用方要 open 则改 cap->p 指向栈槽）、
   关闭（把 *p 拷入 value，含写屏障）、读写。 */
ObjCapVal *obj_capval_new(Gc *gc, Value v);
void obj_capval_close(Gc *gc, ObjCapVal *cap);
void obj_capval_set(Gc *gc, ObjCapVal *cap, Value v);
Value obj_capval_get(const ObjCapVal *cap);

/* 关闭帧内所有开放捕获值并释放该帧的捕获值数组（帧弹出 / context 终结时调用）。
   关闭的 capval 从所属 context 的 opencapval 链摘除（closed 后由闭包持有，不再由 context 保底）。 */
void obj_frame_close_capvals(Gc *gc, mica_context *ctx, CallFrame *cf);

/* 关闭 context 的全部开放捕获值（context 终结/teardown）：遍历 opencapval 链逐个关闭并摘链。
   不读帧 closure/proto（teardown 时 permanent 闭包的 proto 可能已被 finalize 释放），
   也不按帧范围过滤（整个 context 即将释放，全部关闭即正确）。 */
void obj_context_close_all_capvals(Gc *gc, mica_context *ctx);

/* 闭包：包装函数原型 + n 个捕获值（capvals 柔性数组由调用方填充）。 */
ObjClosure *obj_closure_new(Gc *gc, Proto *proto, uint8_t n_capvals);
/* permanent 闭包（普通函数/方法值，capvals 空，编译期阻止捕获）。 */
ObjClosure *obj_closure_new_permanent(Gc *gc, Proto *proto);

/* 错误值：message 是便捷消息；set/case/payload_names/payload 可为 NULL/空（匿名）。
   payload_names 与 payload 由调用方 malloc 提供，所有权移交。 */
ObjError *obj_error_new(Gc *gc, ObjString *message, ObjString *set_name, ObjString *case_name,
                        ObjString **payload_names, Value *payload, size_t n_payload);

/* 枚举值：set/case 名 permanent；payload 由调用方 malloc 提供，所有权移交。 */
ObjEnumVal *obj_enum_val_new(Gc *gc, ObjString *set_name, ObjString *case_name,
                             Value *payload, size_t n_payload);

/* 类 / 实例。 */
ObjClass *obj_class_new_permanent(Gc *gc, ObjString *name);
ObjInstance *obj_instance_new(Gc *gc, ObjClass *klass);

/* 沿继承链查实例方法 / 静态方法 / 属性 getter/setter（name 直接作表键，不创建新字符串）。
   返回 ObjClosure*（permanent 方法闭包）；静态方法返回 Value（ObjClosure 或 ObjNativeClosure）；无则 NULL/null。
   *_kv 变体接受原始键（hash/chars/len），供标识符池（SSO String）直接查找。 */
ObjClosure *obj_class_find_method(ObjClass *klass, ObjString *name);
ObjClosure *obj_class_find_method_kv(ObjClass *klass, uint32_t hash, const char *chars, size_t len);
Value obj_class_find_static(ObjClass *klass, ObjString *name);
Value obj_class_find_static_kv(ObjClass *klass, uint32_t hash, const char *chars, size_t len);
/* getter/setter 查找：返回 ObjClosure（Mica 定义）或 ObjNativeClosure（mica_register_property 注入）；无则 null。 */
Value obj_class_find_getter(ObjClass *klass, ObjString *name);
Value obj_class_find_getter_kv(ObjClass *klass, uint32_t hash, const char *chars, size_t len);
Value obj_class_find_setter(ObjClass *klass, ObjString *name);
Value obj_class_find_setter_kv(ObjClass *klass, uint32_t hash, const char *chars, size_t len);
bool obj_class_has_field(const ObjClass *klass, ObjString *name);
bool obj_class_has_field_kv(const ObjClass *klass, uint32_t hash, const char *chars, size_t len);

/* 实例字段布局（Hidden-Class：柔性数组 + 类上 field_index）。 */
void obj_class_build_field_index(Gc *gc, ObjClass *klass);  /* resolve_super / deser 后调用（父类须已解析） */
int  obj_class_field_index(const ObjClass *klass, const String *name);   /* -1 = 未声明 */
int  obj_class_field_index_kv(const ObjClass *klass, uint32_t hash, const char *chars, size_t len);
Value obj_instance_get_field(const ObjInstance *inst, const String *name);  /* 未声明返回 null */
bool obj_instance_set_field(Gc *gc, ObjInstance *inst, const String *name, Value v); /* false = 未声明 */

/* 注册各 ObjType 的 trace/finalize vtable（gc.c 的 gc_register_vtable）。
   新增对象类型 = 新模块工厂 + 在此注册一次，gc.c 零改动。 */
void obj_vtables_init(Gc *gc);

/* OBJ_CONTEXT finalize（释放栈/帧/defer，关闭开放捕获值）。
   宿主显式释放主 context（mica_context_free）时直接调用，无需等 GC。 */
void obj_context_finalize(Obj *obj);

/* OBJ_CONTEXT 是否协程 context（typeof/is coroutine 用；主 context 非协程报 "context"）。 */
bool obj_context_is_coroutine(const mica_context *ctx);

/* 类型化 Val 便捷函数（避免 ObjString*→Obj* 强转）。
   提升类型（string/array/map/instance/closure）用独立 NaN-box tag，热路径位测试；
   其余对象走 val_obj（MICA_TOBJ）。 */
MICA_FORCEINLINE Value obj_string_val(ObjString *s) { return val_obj_tag(MICA_TSTR, s); }
MICA_FORCEINLINE Value obj_array_val(ObjArray *a) { return val_obj_tag(MICA_TARRAY, a); }
MICA_FORCEINLINE Value obj_map_val(ObjMap *m) { return val_obj_tag(MICA_TMAP, m); }
Value obj_native_closure_val(ObjNativeClosure *n);
Value obj_capval_val(ObjCapVal *cap);
MICA_FORCEINLINE Value obj_closure_val(ObjClosure *cl) { return val_obj_tag(MICA_TCLOSURE, cl); }
Value obj_error_val(ObjError *e);
Value obj_enum_val(ObjEnumVal *e);
Value obj_class_val(ObjClass *c);
MICA_FORCEINLINE Value obj_instance_val(ObjInstance *i) { return val_obj_tag(MICA_TINSTANCE, i); }

/* 内置类型成员方法。 */
ObjBuiltinMethod *obj_builtin_method_new(Gc *gc, mica_native_fn fn);
/* permanent 版本：GC 永不回收。用于 VM 内建方法单例缓存（get_member 热路径每次
   访问 `a.push` / `m.has` 都 new 一个 ObjBuiltinMethod → 每次 GC 分配；改共享
   permanent 单例后消除该分配与 GC 压力）。ObjBuiltinMethod 只含 mica_native_fn fn
   （无状态），共享安全。 */
ObjBuiltinMethod *obj_builtin_method_new_permanent(Gc *gc, mica_native_fn fn);
Value obj_builtin_method_val(ObjBuiltinMethod *bm);

/* 弱引用。 */
ObjWeakRef *obj_weakref_new(Gc *gc, Value target);
Value obj_weakref_val(ObjWeakRef *r);

/* 反射对象构造器已移入 librefl.h（LREFL_* 自定义类型）。 */

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif /* MICA_OBJ_H */
