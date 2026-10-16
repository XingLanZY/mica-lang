#ifndef MICA_LIBREFL_H
#define MICA_LIBREFL_H

#include "obj.h"
#include "value.h"

/* 反射系统（C# 风格，librefl 模块）：typeof() 返回 Type，配套 Assembly/Module/Method/
   Field/Property。作为**原生模块**在 mica_runtime_new 时经 librefl_init 注册：
   - 反射对象类型是**自定义 GC 类型**（LREFL_* = OBJ_TYPE_COUNT+0..5，gc_register_vtable 注册）；
   - 注册 System.Reflection 命名空间（6 类 + Assembly.load）；
   - 注册 `typeof`/`assembly()` 内建；
   - 注册 LREFL_* 类型的成员访问处理器（vm_register_custom_member）。
   import System.Reflection 命中 loaded_or_loading 跳过源码加载。
   本模块对外只暴露 librefl_init（注册接口）；内部实现（构造器/trace/方法/成员分派）
   全部 static，宿主经 mica_register_* 与 mica API 交互。 */

struct mica_runtime;

/* 自定义 GC 类型（在 ObjType 枚举之后；core 文件引用这些常量而非硬编码数值） */
enum {
    LREFL_TYPE = OBJ_TYPE_COUNT,  /* typeof() 返回的 Type 对象 */
    LREFL_METHOD,
    LREFL_FIELD,
    LREFL_PROPERTY,
    LREFL_MODULE,
    LREFL_ASSEMBLY,
    LREFL_COUNT,
};

/* Type 的 prim：决定 name、谓词与结构相等（prim + klass 指针）。
   RTT_NUMBER 覆盖 int/float（底层仍区分 VAL_INT/VAL_FLOAT，类型名统一为 "number"）。
   RTT_CLASS 表示用户类（ObjClass，klass 非空）；其余为内建/反射类型。 */
typedef enum {
    RTT_NULL, RTT_BOOL, RTT_NUMBER,
    RTT_STRING, RTT_ARRAY, RTT_MAP, RTT_FUNCTION, RTT_ERROR,
    RTT_ENUM,             /* 枚举值（ObjEnumVal）：name = 枚举名，结构相等按 name 比较 */
    RTT_COROUTINE, RTT_WEAKREF,
    RTT_CLASS,            /* 用户类（klass 指向 ObjClass，含实例的类） */
    RTT_TYPE, RTT_METHOD, RTT_FIELD, RTT_PROPERTY, RTT_MODULE, RTT_ASSEMBLY,
    RTT_OBJECT,           /* 兜底：内部/未知对象 */
    RTT_COUNT,
} ReflPrim;

typedef struct {
    ObjHeader
    Obj *gclist;               /* gray/gray_again 链（统一偏移） */
    ObjString *name;       /* 类型名：prim==RTT_CLASS 时 = klass->name，否则查 k_prim_names */
    ObjClass *klass;       /* RTT_CLASS 时指向类对象（permanent），否则 NULL */
    uint8_t prim;          /* ReflPrim */
} ObjReflType;

typedef struct {
    ObjHeader
    Obj *gclist;               /* gray/gray_again 链（统一偏移） */
    ObjString *name;       /* 方法名 */
    Value func;            /* 方法函数值（ObjClosure）；协程函数是 ObjClosure（proto->is_coroutine） */
    uint32_t arity;
    bool is_static;        /* 类静态方法 */
    bool needs_receiver;   /* invoke 时首个实参是接收者（类实例方法 true；静态方法/模块函数 false） */
    ObjString *module;     /* 所属模块名（permanent），非模块函数 NULL */
} ObjReflMethod;

typedef struct {
    ObjHeader
    Obj *gclist;               /* gray/gray_again 链（统一偏移） */
    ObjString *name;       /* 字段名 */
} ObjReflField;

typedef struct {
    ObjHeader
    Obj *gclist;               /* gray/gray_again 链（统一偏移） */
    ObjString *name;       /* 属性名 */
    bool has_getter;
    bool has_setter;
    Value getter;          /* getter 函数值（ObjClosure）；无则 null */
    Value setter;          /* setter 函数值（ObjClosure）；无则 null */
} ObjReflProperty;

typedef struct ObjReflAssembly ObjReflAssembly;

typedef struct {
    ObjHeader
    Obj *gclist;               /* gray/gray_again 链（统一偏移） */
    ObjString *name;       /* 模块名（System.IO 等） */
    ObjReflAssembly *assembly; /* 所属程序集（trace） */
} ObjReflModule;

/* 程序集：包装当前 runtime 的 globals + 已加载模块。rt 是进程级指针，**不 trace**。 */
struct ObjReflAssembly {
    ObjHeader
    Obj *gclist;               /* gray/gray_again 链（统一偏移） */
    ObjString *name;       /* 程序集名（入口基名，默认 "main"） */
    mica_runtime *rt;
};

/* 唯一注册接口：mica_runtime_new 调用。注册 System.Reflection 模块 + typeof/assembly
   内建 + LREFL_* 类型 vtable + 成员访问处理器。 */
void librefl_init(mica_runtime *rt);

#endif /* MICA_LIBREFL_H */
