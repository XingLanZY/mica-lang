#ifndef MICA_MICA_H
#define MICA_MICA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ============================================================
   Mica 对外 C API（嵌入宿主程序使用）。
   分层类似 Lua 的 global_State 与 lua_State：
   - mica_runtime：全局状态（GC、全局符号表、内建函数）。一个进程可建多个；
     多个 context 共享同一 runtime 的全局表与 GC。
   - mica_context：一次执行的上下文（编译 + 运行）。须先于 runtime 释放。
   - mica_value：不透明值句柄，只经 mica_* 函数访问。
   ============================================================ */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mica_runtime mica_runtime;
typedef struct mica_context mica_context;

/* 不透明值句柄：固定 8 字节，可按值传递；字段不对外暴露，只能经 mica_* 访问。 */
typedef struct mica_value {
    uint64_t bits[1];
} mica_value;

/* 宿主 native 函数：ctx=调用它的 context；args 是实参（mica_value）；
   写入 results 并返回结果数。创建字符串用 mica_string(ctx 的 runtime, ...)。 */
typedef uint32_t (*mica_native_fn)(mica_context *ctx, mica_value *args, uint32_t argc, mica_value *results);

/* 值类型（mica_value_type） */
enum {
    MICA_NULL = 0,
    MICA_BOOL,
    MICA_INT,
    MICA_FLOAT,
    MICA_STRING,
    MICA_ARRAY,
    MICA_MAP,
    MICA_FUNC,
    MICA_OBJECT,
};

/* ---------- 运行时（全局状态） ---------- */

mica_runtime *mica_runtime_new(void);
void mica_runtime_free(mica_runtime *rt);

/* 窥孔优化开关（字节码优化；默认开，可用环境变量 MICA_PEEPHOLE=0 关闭）。
   注：窥孔在 codegen 阶段运行（无 runtime 引用），故为进程级开关。 */
void mica_set_peephole_enabled(mica_runtime *rt, bool enabled);

/* 读取/写入全局变量（name 作 permanent 键）。字符串值经 mica_string 创建。 */
void mica_runtime_set_global(mica_runtime *rt, const char *name, mica_value v);
mica_value mica_runtime_get_global(mica_runtime *rt, const char *name);

/* 注册宿主 native 函数到全局表（名字作永久键）。 */
void mica_register_native(mica_runtime *rt, const char *name, mica_native_fn fn, uint32_t arity);

/* ---------- 模块实体注册（宿主/原生模块注入，可指定模块） ---------- */

/* 注册一个模块（建命名空间 + 标记已加载，使 import module 跳过源码加载直接解析）。
   各 mica_register_* 会自动注册所属模块，通常无需单独调用。 */
void mica_register_module(mica_runtime *rt, const char *module);

/* 注册一个类（建 ObjClass + 存 globals["module.name"] + 加入命名空间）。
   返回类值（供 mica_register_method/property 用）。 */
mica_value mica_register_class(mica_runtime *rt, const char *module, const char *name);

/* 给类注册方法（is_static=true 静态方法）。fn 是宿主 native（receiver 作首参，静态方法无 receiver）。
   klass 可以是普通类（mica_register_class）或原生类（mica_new_native_class）。 */
void mica_register_method(mica_runtime *rt, mica_value klass, const char *name,
                          mica_native_fn fn, uint32_t arity, bool is_static);

/* 给普通类（mica_register_class 创建）注册声明字段。字段编译期固定、实例布局继承前缀，
   须在创建任何实例之前调用（后注册会重建 field_index，已建实例字段布局失效）。 */
void mica_register_field(mica_runtime *rt, mica_value klass, const char *name);

/* ---------- 原生类（绑定 C 结构体 userdata；QuickJS JSClassDef/JS_NewClass 风格） ----------
   Mica 普通类与原生类不同：普通类实例是 Mica 声明字段（Value[]）；原生类实例是宿主
   C 结构体（opaque userdata 字节），GC 无法追踪其内容，只能经 finalizer 在回收时释放。 */

/* 原生实例回收回调（释放 C 结构体资源；可为 NULL）。 */
typedef void (*mica_native_finalizer)(void *userdata);

/* 原生类定义（QuickJS JSClassDef 类似物）。 */
typedef struct mica_native_class_def {
    const char *name;                /* 类名（挂在 module 命名空间下） */
    size_t userdata_size;            /* 每实例 C 结构体字节数 */
    mica_native_finalizer finalizer; /* 实例回收回调（可为 NULL） */
} mica_native_class_def;

/* 注册原生类（QuickJS JS_NewClass 类似物）：建 ObjNativeClass + 存 globals["module.name"]。
   幂等：同名已注册返回已有类。返回类值（供 mica_register_method 注入方法）。 */
mica_value mica_new_native_class(mica_runtime *rt, const char *module,
                                 const mica_native_class_def *def);

/* 创建原生类实例（QuickJS JS_NewObjectClass 类似物）：userdata 零初始化。
   在 Mica 脚本里即 `NativeClass()`。 */
mica_value mica_new_native_instance(mica_context *ctx, mica_value klass);

/* 取原生实例的 userdata 指针（QuickJS JS_GetOpaque 类似物）：obj 是原生实例时返回
   C 结构体指针（写入大小到 *size），否则返回 NULL。宿主 native 内取 C 结构体用。 */
void *mica_native_userdata(mica_context *ctx, mica_value obj, size_t *size);

/* 给类注册属性（getter/setter 原生；任一侧可为 NULL 表示只读/只写）。 */
void mica_register_property(mica_runtime *rt, mica_value klass, const char *name,
                            mica_native_fn getter, mica_native_fn setter);

/* 注册模块级函数（存 globals["module.name"] + 命名空间）。 */
void mica_register_function(mica_runtime *rt, const char *module, const char *name,
                            mica_native_fn fn, uint32_t arity);

/* 注册模块级常量。 */
void mica_register_constant(mica_runtime *rt, const char *module, const char *name, mica_value v);

/* 注册枚举（成员名→number；values 为 NULL 时自动 0..n-1）。 */
void mica_register_enum(mica_runtime *rt, const char *module, const char *name,
                        const char *const *members, const int64_t *values, size_t n);

/* context 所属的 runtime（native 内取 runtime 用）。 */
mica_runtime *mica_context_runtime(mica_context *ctx);

/* 调用已定义在全局表中的 mica 函数（须先 mica_compile/mica_run 定义）。
   0=成功；result 可为 NULL。错误用 mica_error_message 查。 */
int mica_call(mica_context *ctx, const char *name, const mica_value *args, int argc, mica_value *result);

/* ---------- 上下文（执行） ---------- */

mica_context *mica_context_new(mica_runtime *rt);
void mica_context_free(mica_context *ctx);

/* 编译源码为可执行程序（0=成功）；随后 mica_run 执行。
   再次 mica_compile 会替换当前程序。 */
int mica_compile(mica_context *ctx, const char *src, size_t len);
int mica_run(mica_context *ctx);
/* 编译并立即运行（0=成功）。 */
int mica_dostring(mica_context *ctx, const char *src, size_t len);

/* 最近一次错误（编译期或运行期）的描述与行号；无错误返回 "" / 0。 */
const char *mica_error_message(const mica_context *ctx);
int mica_error_line(const mica_context *ctx);
/* 出错文件路径：模块加载失败时指向模块文件，否则为 NULL（用入口路径）。 */
const char *mica_error_path(const mica_context *ctx);

/* ---------- 模块系统 ---------- */

/* 添加模块搜索目录。模块名 System.IO 会尝试 <dir>/System/IO.mica 与 <dir>/System.IO.mica。 */
void mica_runtime_add_search_path(mica_runtime *rt, const char *dir);

/* 加载并编译模块（含其 import 递归初始化）。0=成功；失败用 mica_runtime_error 查详情。 */
int mica_load_module(mica_runtime *rt, const char *name);
const char *mica_runtime_error(const mica_runtime *rt);
/* 最近一次模块加载失败的文件路径；无失败返回 ""。 */
const char *mica_runtime_error_path(const mica_runtime *rt);

/* 编译并运行入口文件（含其 import；入口 = 顶层语句 或 main()）。 */
int mica_dofile(mica_context *ctx, const char *path);

/* ---------- 程序集（assembly） ---------- */

/* 编译程序集：入口 + 所有 import 模块 → 单个二进制文件（类似 C# 程序集 / dex）。
   entry_path 所在目录自动加入模块搜索路径。0=成功。 */
int mica_compile_assembly(mica_runtime *rt, const char *entry_path, const char *out_path);

/* 编译库：把单个模块文件（export module）编译为 .mlib，供其它代码 import。
   0=成功。 */
int mica_compile_library(mica_runtime *rt, const char *module_path, const char *out_path);

/* 加载程序集：反序列化重建模块 init + 入口并执行模块 init（注册命名空间），
   随后 mica_run 可运行入口。0=成功。 */
int mica_load_assembly(mica_context *ctx, const char *path);

/* 运行时动态加载库程序集（.mlib）：反序列化 + 按依赖序运行模块 init + 注册命名空间。
   加载后模块可从反射 Assembly.module()/get_type() 访问（不改变当前程序集名）。
   重复加载同一模块会被跳过。0=成功；失败用 mica_runtime_error 查详情。 */
int mica_runtime_load_library(mica_runtime *rt, const char *path);

/* ---------- 值 ---------- */

mica_value mica_null(void);
mica_value mica_bool(int b);
mica_value mica_int(int64_t i);
mica_value mica_float(double f);
/* 创建字符串值（在 runtime 的 GC 中分配；宿主应尽快存入全局表或立即使用）。 */
mica_value mica_string(mica_runtime *rt, const char *s, size_t len);

int mica_value_type(const mica_value *v);
bool mica_is_null(mica_value v);
bool mica_is_bool(mica_value v);
bool mica_is_int(mica_value v);
bool mica_is_float(mica_value v);
bool mica_is_string(mica_value v);

bool mica_as_bool(mica_value v);
int64_t mica_as_int(mica_value v);
double mica_as_float(mica_value v);
const char *mica_as_string(mica_value v, size_t *len); /* NULL 表示非字符串 */

void mica_print_value(mica_value v);   /* 打印到 stdout（调试） */

/* ---------- 协程 C API（Phase 2c/2e：宿主 native 挂起 + 宿主驱动协程） ---------- */

/* 续体：resume 时被解释器调用。ud = native 传给它的状态指针（native 的中途 C 状态
   迁到堆对象，经 ud 拿回）。results 写续体结果（协程恢复值），返回结果数。 */
typedef uint32_t (*mica_continuation)(mica_context *ctx, void *ud,
                                      const mica_value *resume_args, uint32_t n,
                                      mica_value *results);

/* 挂起当前协程（native 内调用后正常 return）：产出 yield_value；resume 时以恢复实参
   调 cont（其结果为恢复值）。cont==NULL → resume 后协程直接完成。无 longjmp。 */
void mica_yield(mica_context *ctx, mica_value yield_value, mica_continuation cont, void *ud);

/* 调 Mica 闭包 fn（args[0..argc)，argc 含 self 槽）。若内层 yield：自动注册 k+ud 为续体、
   置 yield_requested、透传内层产出，返回 MICA_CALL_YIELDED；否则正常执行，
   results[0]=返回值，返回 1。 */
#define MICA_CALL_YIELDED (-1)
int mica_callk(mica_context *ctx, mica_value fn, mica_value *args, uint32_t argc,
               mica_continuation k, void *ud, mica_value *results);

/* 宿主从外部驱动协程（等价 coroutine.create/resume/cancel/status）。co 是
   coroutine.create 返回的协程值（OBJ_CONTEXT）。 */
mica_value mica_coroutine_create(mica_context *ctx, mica_value fn);

/* resume 协程 co：args[0..argc) 是 resume 实参（首次=启动参数；之后=yield 恢复参数）。
   返回：MICA_RESUME_DONE（*result=完成值）/ MICA_RESUME_YIELD（*result=产出值）/
   MICA_RESUME_ERROR（运行期错误 / resume DEAD/RUNNING 协程）。 */
#define MICA_RESUME_DONE   0
#define MICA_RESUME_YIELD  1
#define MICA_RESUME_ERROR  2
int mica_resume(mica_context *ctx, mica_value co, const mica_value *args, int argc,
                mica_value *result);

/* 协程状态（对齐 Lua coroutine.status）。 */
#define MICA_CORO_SUSPENDED 0
#define MICA_CORO_RUNNING   1
#define MICA_CORO_NORMAL    2
#define MICA_CORO_DEAD      3
int mica_coroutine_status(mica_context *ctx, mica_value co);

/* 取消协程：跑 pending defer + 关 capval + 立即释放 context 内存。之后 resume 报错。 */
int mica_coroutine_cancel(mica_context *ctx, mica_value co);

#ifdef __cplusplus
}
#endif

#endif /* MICA_MICA_H */
