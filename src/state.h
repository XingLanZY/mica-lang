#ifndef MICA_STATE_H
#define MICA_STATE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <setjmp.h>

#include "mica.h"   /* mica_runtime / mica_context 不透明 typedef */
#include "gc.h"
#include "peephole.h" /* PeepholeState（runtime 持有） */
#include "obj.h"
#include "proto.h"  /* Instr（CallFrame.savedpc） */
#include "table.h"

/* ============================================================
   引擎内部结构（供 mica.c / vm.c / builtin.c / assembly.c 等共享）。
   - mica.h 是对外公开接口（不透明类型）；本文件是内部实现细节。
   - mica_context = 一个执行线程/协程（类 Lua 的 lua_State）：
     主程序与每个协程各是一个 mica_context，直接持有栈/帧/defer。
   - mica_runtime = 进程级全局状态（GC、全局表、模块表），可被多个 context 共享。
   ============================================================ */

/* mica_runtime：进程级全局状态（GC + 全局符号表 + 模块表）。 */

/* 运算符重载方法名缓存索引（与 vm.c 的 k_op_name_lits 一一对应）。
   惰性创建 permanent ObjString，避免每调用 obj_string_new 分配查找键。 */
enum {
    OP_NAME_ADD, OP_NAME_SUB, OP_NAME_MUL, OP_NAME_DIV, OP_NAME_IDIV,
    OP_NAME_MOD, OP_NAME_POW, OP_NAME_NEG, OP_NAME_LT, OP_NAME_LE,
    OP_NAME_GT, OP_NAME_GE, OP_NAME_EQ, OP_NAME_NE, OP_NAME_INDEX,
    OP_NAME_INDEX_SET, OP_NAME_COUNT
};

struct mica_runtime {
    Gc gc;                        /* 共享垃圾回收器（内嵌值；ctx->gc 指向 &rt->gc） */
    PeepholeState peep;           /* 窥孔优化器状态（按 runtime 隔离，非进程级全局） */
    ObjTable *globals;            /* permanent 根对象：全局符号表 */
    const char **search_path;     /* 模块搜索目录（dup_str，释放时逐个 free） */
    size_t n_search, cap_search;
    const char **loaded_modules;  /* 已加载模块名（防重复初始化 / 循环导入） */
    size_t n_loaded, cap_loaded;
    ObjTable *module_inits;       /* 模块名 → init ObjClosure（程序集编译收集用） */
    ObjTable *module_exports;     /* 模块名 → ObjArray[ObjString] 导出名（命名空间重建用） */
    ObjString *init_name;         /* 缓存 "init" permanent 串（构造器查找键；须随 runtime 存活，勿用函数级 static） */
    ObjString *op_names[OP_NAME_COUNT]; /* 运算符重载方法名 permanent 缓存（vm.c 惰性填充） */
    ObjClass *weakref_class;      /* WeakRef 类对象（permanent，native_ctor；extern class 注入用） */
    /* 反射类（Type/Assembly/...）由 librefl 模块注册进 System.Reflection 命名空间，无 runtime 字段 */
    char *assembly_name;          /* 程序集名（入口基名，默认 "main"；Assembly.name 用）。malloc，rt free 释放 */
    char *last_error;             /* 最近一次模块加载/编译错误消息。malloc，rt free 释放 */
    char *last_error_path;        /* 最近一次模块加载失败的文件路径。malloc，rt free 释放；无路径 NULL */
    int last_error_line;
};

/* 值栈字段直接内联在 mica_context（2026-08-11 去 MicaStack 结构体）：
   stack = 堆数组基址（可 realloc 增长），stack_size = 容量（Value 数），
   stack_top = 栈顶指针（= stack + 已用槽数）。realloc 后所有栈内指针
   （fr->base / fr->u.getter.getter_base / ctx->stack_top / opencapval->p）
   按偏移（新基址 - 旧基址）更新。 */

/* 调用帧（Lua CallInfo 风格，2026-08-09 精简至 48B）：独立帧数组 + savedpc 指令指针。
   - 执行 proto = closure->proto（不单独存 func 指针）。
   - 开放捕获值不挂在帧上：ctx->opencapval 链（Lua luaF_findupval 风格）是唯一数据源。
   - union u 把互斥的"构造器帧"（defaults→init 链）和"属性 getter/setter 帧"字段共享；
     构造器的 cargc/cwant 平铺（仅构造器帧有效）。
   - savedpc 指向 proto->code 中"下一条"指令，主循环 `*fr->savedpc++` 取指令。
   - base 是栈内指针（Value*），栈 realloc 时由 ensure_stack 按偏移更新。 */
typedef struct CallFrame {
    ObjClosure *closure;      /* 闭包（capvals 访问）；执行 proto = closure->proto */
    Instr *savedpc;           /* 指令指针：指向 closure->proto->code 中"下一条"要执行的指令 */
    union {                   /* 互斥字段（Lua CallInfo 风格） */
        struct {              /* 构造器帧（defaults→init 链）：仅构造器用 */
            ObjClosure *after;        /* defaults 返回后要执行的 init */
            ObjInstance *constructed; /* 正在构造的实例 */
        } ctor;
        struct {              /* 属性 getter/setter 帧：仅属性访问用 */
            int getter_dest;  /* getter 结果写回父帧寄存器偏移；setter 用 -1 */
            Value *getter_base; /* 父帧栈基址指针（栈 realloc 时按偏移更新） */
        } getter;
    } u;
    Value *base;              /* 帧栈基址指针（slot0=函数值，参数 1..arity；realloc 后按偏移更新） */
    uint8_t  cwant;           /* 构造器：结果模式（RESULT_DISCARD / 固定，仅 ctor 帧有效） */
    uint8_t  flags;           /* 位标志：CF_DEFER / CF_GETTER */
    uint8_t  want;            /* 结果模式（RESULT_DISCARD / RESULT_VARARGS / 固定 1..126） */
    uint8_t  pad;             /* 对齐 */
    uint16_t cargc;           /* 构造器：defaults→init 实参数（仅 ctor 帧有效） */
    uint16_t argc;            /* 调用时实际实参数（vararg 取多余用） */
    int defer_mark;           /* 本帧在 ctx->defers 的起始下标 */
} CallFrame;

#define CF_DEFER   (1u << 0)  /* 本帧是 defer 闭包执行帧 */
#define CF_GETTER  (1u << 1)  /* 本帧是属性 getter/setter 帧 */
#define CF_UNPROTECTED (1u << 2) /* 本帧由未受 try/catch 保护的调用压入：返回错误值 → 未捕获错误退出 */

/* 帧存储：帧块（FrameBlock）链表（弹性，pointer-stable）。
   - 每块 FRAME_BLOCK_N 帧，链式增长；块分配后不移动 → 已持有的 fr 指针永不失效。
   - 块不立即释放（防振荡 realloc）：弹空后留在链上，再次压入同区自动复用。
   - 访问约定：ctx_frame(ctx) = 顶帧；遍历 = 逐块扫 used。
   - 软上限 MICA_FRAME_MAX_SOFT 防失控递归（对齐 Lua LUAI_MAXCCALLS 精神，允许更深）。 */
#define FRAME_BLOCK_N 256
#define MICA_FRAME_MAX_SOFT (1 << 20)

typedef struct FrameBlock {
    struct FrameBlock *next;  /* 下一块（更高帧下标） */
    struct FrameBlock *prev;  /* 上一块（更低帧下标） */
    int first;                /* 本块第一个帧的全局下标（= 块序 * FRAME_BLOCK_N） */
    int used;                 /* 本块活跃帧数（非末块恒为 FRAME_BLOCK_N；末块实时维护） */
    CallFrame frames[FRAME_BLOCK_N];
} FrameBlock;

/* mica_context：一次编译+运行的执行上下文；也是一个执行线程/协程。
   可 GC 对象（OBJ_CONTEXT）：嵌入 Obj 头，vtable 的 trace 枚举 defer 存活值，
   finalize 释放栈/帧/defer 缓冲。直接持有执行状态，无独立 CoroutineState。
   ⚠️ 精简原则（2026-08-15 对照 Lua lua_State）：①单 `status` 字节（Lua TStatus）
   合一协程态/错误位，删 is_coroutine/coro_status/has_error 三字段；②`globals` 由
   `rt->globals` 推导（Lua 经 G(L)->gt 两跳），删缓存字段；③`error_msg/error_line`
   并入 `st_error_msg/st_error_line`（API 直接读运行期错误，删拷贝）；④`coro_started`
   由 `frame_count==0` 推导。 */
struct mica_context {
    ObjHeader                 /* GC 对象头（OBJ_CONTEXT） */
    Obj *gclist;              /* gray/gray_again 链（统一偏移，见 obj.h GcGCNode） */
    Obj *ctx_next;            /* GC contexts 链（运行期协程 context；主 context 是 permanent 不入链） */
    mica_runtime *rt;         /* 所属 runtime（共享 gc/globals） */
    Gc *gc;                   /* = &rt->gc（热路径缓存，同 Lua l_State.l_G） */
    ObjClosure *program;       /* 最近编译的程序（permanent 于 &rt->gc；协程为 NULL） */
    char *error_path;         /* 出错文件：模块加载失败时指向模块文件。malloc；无路径 NULL */

    /* ---- 执行状态（Lua 风格约定：slot0=函数值，参数 1..arity） ---- */
    Value *stack;             /* 值栈基址（可 realloc 增长；增长后按偏移更新所有栈内指针） */
    int stack_size;           /* 栈容量（Value 数） */
    Value *stack_top;         /* 栈顶指针（= stack + 已用槽数；realloc 后按偏移更新） */
    ObjCapVal *opencapval;    /* 本 context 所有开放捕获值链（Lua 的 lua_State.openupval 等价）。
                                 OP_CLOSURE open 时头插；帧弹出 close 时摘除。
                                 context_trace 遍历它保底 open capval（帧弹出仍要 close）。 */
    FrameBlock *frame_blocks; /* 帧块链表头（pointer-stable；空栈时 NULL） */
    FrameBlock *cur_block;    /* 当前活跃块（顶帧所在；空栈时=头部块，压帧前复用） */
    int frame_count;
    /* Lua TStatus 风格单字节状态：低 4 位协程态（CORO_NONE=主 context / SUSPENDED /
       RUNNING / NORMAL / DEAD），bit7 = 错误位（STATUS_ERROR）。见下方访问宏。 */
    uint8_t status;
    char *st_error_msg;       /* 运行期错误消息（API mica_error_message 直读）。malloc，释放时 free */
    int st_error_line;
    Value st_error_value;     /* 运行期错误原对象（Phase 2g 保真；无=null）。context_trace 扫。 */
    jmp_buf *active_jmp;      /* 当前活跃 vm_run 的错误跳转缓冲（非执行期 NULL，供 runtime_error
                                 longjmp 展开；不是 GC Value，context_trace 不碰） */
    /* 协程状态（仅协程 context 有效；值=OBJ_CONTEXT 时由 typeof/is coroutine 报告） */
    ObjClosure *coro_closure;  /* 协程体闭包（GC-trace；启动帧 slot0 也持引用） */
    Value *coro_start_args;    /* 首次 resume 的启动参数（malloc；消费后 NULL） */
    int n_coro_start_args;
    uint8_t resume_depth;      /* 嵌套 resume 深度护栏（MICA_RESUME_MAX_DEPTH=200，<256） */
    /* Phase 2c：native 请求挂起（mica_yield/mica_callk） */
    bool yield_requested;      /* native 调 mica_yield 后置位：解释器检测 → 挂起 */
    Value yield_value;         /* 挂起产出值 */
    mica_continuation yield_cont; /* resume 时调用的续体（NULL=resume 后协程直接完成） */
    void *yield_ud;            /* 续体 ud（GC 保根由调用方负责：ud 须含 Mica Value 时做成 GC 对象或存栈） */
    uint8_t yield_res_reg;     /* 挂起 native 调用的结果寄存器（续体结果写入处，相对帧 base，<256） */
    Value *defers;            /* 变长 defer 栈（闭包 + null 标记） */
    int defer_cap;
    int defer_top;
};

/* ---- 单字节 status（Lua TStatus 风格）访问宏 ----
   `coro_status` 低 4 位 + `has_error` bit7 合一。主 context 协程态 = CORO_NONE。 */
#define STATUS_ERROR 0x80u
#define CTX_IS_CORO(ctx)        (((ctx)->status & 0x0Fu) != CORO_NONE)
#define CTX_CORO_STATUS(ctx)    ((ctx)->status & 0x0Fu)
#define CTX_SET_CORO(ctx, s)    ((ctx)->status = (uint8_t)(((ctx)->status & (uint8_t)~0x0Fu) | (s)))
#define CTX_HAS_ERROR(ctx)      (((ctx)->status & STATUS_ERROR) != 0)
#define CTX_SET_ERROR(ctx)      ((ctx)->status = (uint8_t)((ctx)->status | STATUS_ERROR))
#define CTX_CLR_ERROR(ctx)      ((ctx)->status = (uint8_t)((ctx)->status & (uint8_t)~STATUS_ERROR))

/* globals 由 rt 推导（删缓存字段，同 Lua G(L)->gt）。 */
#define ctx_globals(ctx) ((ctx)->rt->globals)

/* ---------- 帧块辅助（热路径内联；块链 pointer-stable） ----------
   - ctx_frame(ctx)：顶帧指针（frame_count > 0 时有效；vm_run 主循环 fr 缓存源）。
   - ctx_frame_alloc(ctx)：为下一帧分配槽位（frame_count 不变）。跨块时沿链找
     first==i 的既有块复用，否则在链尾分配新块并推进 cur_block。
   - ctx_frame_new(ctx)：alloc + frame_count++ + 更新 used，返回槽位。
   - ctx_frame_pop(ctx)：frame_count-- + 块管理。弹空的块留链（used=0），下次
     压入同区自动复用；空栈时 cur_block 回落头部块（供下次 alloc 复用）。 */

MICA_FORCEINLINE CallFrame *ctx_frame(mica_context *ctx) {
    return &ctx->cur_block->frames[ctx->frame_count - 1 - ctx->cur_block->first];
}

/* 首帧指针（frame_count > 0 时有效）：块 0 恒存首帧（first==0）。 */
MICA_FORCEINLINE CallFrame *ctx_frame_first(mica_context *ctx) {
    return &ctx->frame_blocks->frames[0];
}

MICA_FORCEINLINE CallFrame *ctx_frame_alloc(mica_context *ctx) {
    const int i = ctx->frame_count;
    if (MICA_UNLIKELY((i & (FRAME_BLOCK_N - 1)) == 0)) {
        FrameBlock *b = ctx->frame_blocks;
        while (b && b->first != i) b = b->next;
        if (!b) {
            FrameBlock *tail = ctx->frame_blocks;
            while (tail && tail->next) tail = tail->next;
            b = (FrameBlock *)calloc(1, sizeof(FrameBlock));
            assert(b != NULL);
            b->first = i;
            b->prev = tail;
            if (tail) tail->next = b;
            else ctx->frame_blocks = b;
        }
        ctx->cur_block = b;
    }
    return &ctx->cur_block->frames[i - ctx->cur_block->first];
}

MICA_FORCEINLINE CallFrame *ctx_frame_new(mica_context *ctx) {
    CallFrame *f = ctx_frame_alloc(ctx);
    ctx->frame_count++;
    ctx->cur_block->used = ctx->frame_count - ctx->cur_block->first;
    return f;
}

MICA_FORCEINLINE void ctx_frame_pop(mica_context *ctx) {
    ctx->frame_count--;
    if (MICA_UNLIKELY(ctx->frame_count == 0)) {
        ctx->cur_block->used = 0;
        ctx->cur_block = ctx->frame_blocks; /* 空栈：回落头部块，下次压帧复用块 0 */
        return;
    }
    if (ctx->frame_count <= ctx->cur_block->first) {
        ctx->cur_block->used = 0;
        ctx->cur_block = ctx->cur_block->prev;
    } else {
        ctx->cur_block->used = ctx->frame_count - ctx->cur_block->first;
    }
}

/* 帧块链遍历辅助（GC trace / 批量关闭 capval / cancel 等冷路径）：
   for (FrameBlock *b = ctx->frame_blocks; b; b = b->next)
       for (int i = 0; i < b->used; i++) { CallFrame *f = &b->frames[i]; ... }
   used 由 ctx_frame_new/pop 实时维护（末块外恒 FRAME_BLOCK_N）。 */

/* 释放整个帧块链（context 销毁时）。 */
MICA_FORCEINLINE void ctx_frame_blocks_free(mica_context *ctx) {
    FrameBlock *b = ctx->frame_blocks;
    while (b) {
        FrameBlock *nx = b->next;
        free(b);
        b = nx;
    }
    ctx->frame_blocks = NULL;
    ctx->cur_block = NULL;
}

/* 内部：按给定栈大小创建 context（主程序用 MICA_STACK_SIZE；协程用 MICA_CORO_STACK_INIT）。
   is_main=true → permanent 对象（permanent 链作根，宿主持有，mica_context_free 显式释放）。 */
mica_context *mica_context_new_sized(mica_runtime *rt, size_t stack_size, bool is_main);

/* 索引化全局：解析 proto 的 LOAD_GLOBAL/STORE_GLOBAL ident 下标 → globals 表条目指针缓存。
   proto.c 实现（首访/表版本变化时调用）。 */
void proto_resolve_globals(const ObjTable *globals, Proto *ch);

#endif /* MICA_STATE_H */
