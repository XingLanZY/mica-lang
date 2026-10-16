#ifndef MICA_VM_H
#define MICA_VM_H

#include <stdarg.h>
#include <stdint.h>

#include "opcodes.h"   /* Instr（不再定义 uint32_t Instr；proto.h 依赖 obj/value/str/table，vm.h 只引 opcodes.h） */
#include "obj.h"
#include "value.h"

/* ---------- computed goto 直接线程化（GCC/Clang 支持，MSVC 不支持） ----------
   VM_COMPUTED_GOTO=1 时 vm_run 主循环用 `goto *分发表[op]` 直接线程化，省 switch
   跳转表的索引间接跳转 + 每迭代回环（热 op 处理代码前后相连，i-cache 友好）；
   MSVC 无 `&&label`/`goto *ptr`，回退 switch 路径。两模式共享同一套 case 代码：
   VM_CASE(op)    switch 展开 `case OP_##op:`；CG 展开线程化标签 `L_VM_OP_##op:`
   VM_DISPATCH()  switch 展开 `break`（回 while 循环顶取指令）；CG 取下一条指令
                       并 `goto *k_vm_targets[op]`。CG 模式要求 vm_run 内有函数级
                       `Instr in;`、`fr`，及静态分发表 `k_vm_targets`。
   VM_DEFAULT()   switch 展开 `default:`；CG 展开 `L_VM_DEFAULT:` */
/* CG 模式要求 vm_run 内有函数级 `Instr in;`、局部指令指针 `Instr *pc`、`fr`，
   及静态分发表 `k_vm_targets`。pc 是热分派源（`in = *pc++`），fr->savedpc 只在
   帧边界/挂起点写回（Lua savepc 模式）。 */
/* 编译器检测：GCC/Clang 支持 `&&label`/`goto *ptr` → 直接线程化；MSVC 回退 switch。
   可用编译期 `-DVM_FORCE_SWITCH=1` 强制回退 switch（调试 / A-B 对比）。 */
#if defined(VM_FORCE_SWITCH) && VM_FORCE_SWITCH
#define VM_COMPUTED_GOTO 0
#elif defined(__GNUC__) || defined(__clang__)
#define VM_COMPUTED_GOTO 1
#else
#define VM_COMPUTED_GOTO 0
#endif

#if VM_COMPUTED_GOTO
#define VM_CASE(op)   L_VM_OP_##op:
#define VM_DISPATCH() do { \
        in = *pc++; \
        goto *k_vm_targets[instr_op(in)]; } while (0)
#define VM_DEFAULT()  L_VM_DEFAULT:
#else
#define VM_CASE(op)   case OP_##op:
/* switch 模式：VM_DISPATCH 保持纯 `break`（`if (cond) VM_DISPATCH(); <后续>`
   模式依赖 break 是 if 体的一部分——注入独立语句会悬空 break 破坏控制流）。 */
#define VM_DISPATCH() break
#define VM_DEFAULT()  default:
#endif

/* 寄存器 VM 执行引擎（实现在 vm.c）。
   VM 是纯函数，不做独立结构：直接操作 mica_context（经 state.h 访问其字段）。
   - 对外 C API 只在 mica.h（mica_context 不透明给宿主）。
   - runtime_error/make_error 实现在 mica.c（builtin.c 与 VM 共用）。 */
typedef struct mica_context mica_context;

/* vm_run 的结果 */
typedef enum { CORO_DONE, CORO_ERROR, CORO_YIELDED } CoroResult;

/* 主程序 Value 栈大小（mica_context_new 分配；vm.c 栈溢出检查用）。 */
#define MICA_STACK_SIZE 65536 /* 主程序 Value 栈初始大小（可动态增长） */
#define MICA_STACK_MAX (1 << 20) /* 栈硬上限（槽数，约 16MB） */

/* 运行 context 直到：帧数降到 stop_count（返回 CORO_DONE 且 *out=首结果）、
   协程 yield（CORO_YIELDED 且 *out=产出值）、或运行期错误（CORO_ERROR）。
   调用方负责设置初始帧与错误复位。
   注意：*out 只在 stop_count==0（帧数归零）时由 vm_run 填充；
   嵌套运行（stop_count>0，如运算符重载）要自行从帧 base 槽读返回值。 */
CoroResult vm_run(mica_context *ctx, Value *out, int stop_count);

/* 协程引擎（Phase 1.5+，供 libcoro / mica_resume 调用）：
   - vm_coroutine_run：resume 协程 cctx。返回 CORO_YIELDED/DONE/ERROR；
     ERROR 时 cctx->st_error_msg/st_error_value 有效（不抛到 ctx）。
   - vm_coroutine_cancel：取消协程（Lua close 语义：跑 defer + 释放执行态）。 */
int vm_coroutine_run(mica_context *ctx, mica_context *cctx, const Value *args, int nargs,
                     Value *out);
void vm_coroutine_cancel(mica_context *ctx, mica_context *cctx);

/* ---------- 编译期格式串检查 ----------
   这些函数是变参的，格式串与实参不匹配时不会有任何编译期提示（历史上出现过把
   `a->count` 写成数组值、把指针当 %zu 打印的静默错误）。加 format 属性后编译器
   会直接报错。MSVC 用 _Printf_format_string_ + SAL，GCC/Clang 用 __attribute__。 */
#if defined(__GNUC__) || defined(__clang__)
#define MICA_PRINTF(fmt_idx, arg_idx) __attribute__((format(printf, fmt_idx, arg_idx)))
#else
#define MICA_PRINTF(fmt_idx, arg_idx)
#endif

/* VM 运行时共享工具（builtin.c 也使用）： */
/* 运行期错误：设置当前 context 的 error（fr 可为 NULL，仅用于行号定位）。 */
MICA_PRINTF(3, 4)
void runtime_error(mica_context *ctx, CallFrame *fr, const char *fmt, ...);
/* 运行期错误（显式 pc 版）：行号算自 pc（pc 指向"当前指令之后"，同 savedpc 语义）。
   vm_run 主循环热路径用局部 pc 直接报错，免写回 fr->savedpc。 */
MICA_PRINTF(4, 5)
void runtime_error_at(mica_context *ctx, CallFrame *fr, const Instr *pc, const char *fmt, ...);
/* 构造匿名错误值（消息字符串）。 */
Value make_error(mica_context *ctx, const char *msg);
/* 设置 context 运行期错误消息（无 longjmp；协程引擎预检错误等"以值返回"路径用）。 */
MICA_PRINTF(2, 3)
void ctx_set_error(mica_context *ctx, const char *fmt, ...);
/* 内部工具（libcoro 等扩展模块用）：栈增长（Lua 式动态栈）与错误重抛。 */
bool vm_ensure_stack(mica_context *ctx, const Value *need_top);
void vm_rethrow(mica_context *ctx);
/* 把任意值转成 C 字符串（malloc 返回，调用方 free）。 */
char *value_to_cstr(Value v);

/* 从 native/builtin 上下文调用函数值（ObjClosure/ObjNativeClosure）。
   嵌套帧建在当前帧寄存器顶之上；返回 true=已处理（错误时 *out=null），
   false=不可调用（错误已置）。self=显式 receiver（普通函数传 null）。
   统一约定：slot0=func、slot1=self、slot2+=实参。反射 Method.invoke / Property.get/set 用。 */
bool vm_call_value(mica_context *ctx, CallFrame *fr, Value fn, Value self, const Value *args,
                   int nargs, Value *out);

/* 自定义类型（>= OBJ_TYPE_COUNT）成员访问处理器：lib 模块（如 librefl 反射对象）注册，
   vm 的 get_member 对自定义类型经它分派。 */
typedef Value (*VmCustomMemberFn)(mica_context *ctx, CallFrame *fr, Value obj, const String *name);
void vm_register_custom_member(uint32_t type, VmCustomMemberFn fn);

#endif /* MICA_VM_H */
