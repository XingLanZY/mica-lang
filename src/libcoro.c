/* libcoro.c —— 协程标准库模块（coroutine 命名空间 + 协程 C API）。
   对象模型（Phase 1.5）：协程实例 = mica_context（OBJ_CONTEXT，协程值）；
   工厂 = 普通 ObjClosure（Proto.is_coroutine）。语言 API 收敛在 coroutine 命名空间：
     coroutine.create(fn, ...)   从闭包建协程（返回 OBJ_CONTEXT 值），... 为启动参数
     coroutine.resume(co, ...)   resume；返回 yield/完成值；出错按 Mica 错误模型（值）传播
     coroutine.cancel(co)        cancel（Lua close 语义：跑 defer + 释放执行态）
     coroutine.status(co)        "suspended"/"running"/"normal"/"dead"
   底层引擎在 vm.c（vm_coroutine_run/vm_coroutine_cancel），本模块只做参数转换/注册。 */

#include <stdlib.h>
#include <string.h>

#include "mica.h"
#include "state.h"
#include "vm.h"
#include "obj.h"
#include "value.h"

/* 宿主 native 结果 staging 容量（与 mica.c 的 MICA_NATIVE_MAX_RESULTS 一致）。 */
#define MICA_NATIVE_MAX_RESULTS 8

/* ---------- 参数校验辅助 ---------- */

/* 从 native 实参取协程 context（args[0] 必须是 OBJ_CONTEXT 协程；否则报错返回 NULL）。 */
static mica_context *coro_arg(mica_context *ctx, mica_value *args, uint32_t argc,
                              const char *fn_name) {
    if (argc < 1) {
        runtime_error(ctx, NULL, "%s expects a coroutine", fn_name);
        return NULL;
    }
    const Value cov = val_unwrap(args[0]);
    if (!val_is_obj(cov) || val_as_obj(cov)->type != OBJ_CONTEXT ||
        !CTX_IS_CORO((mica_context *)val_as_obj(cov))) {
        runtime_error(ctx, NULL, "%s expects a coroutine", fn_name);
        return NULL;
    }
    return (mica_context *)val_as_obj(cov);
}

/* ---------- coroutine.create ---------- */

static uint32_t native_coroutine_create(mica_context *ctx, mica_value *args, uint32_t argc,
                                        mica_value *results) {
    if (argc < 1) {
        runtime_error(ctx, NULL, "coroutine.create expects a function");
        return 0;
    }
    const Value fn = val_unwrap(args[0]);
    if (!val_is_obj(fn) || val_as_obj(fn)->type != OBJ_CLOSURE) {
        runtime_error(ctx, NULL, "coroutine.create expects a function");
        return 0;
    }
    ObjClosure *cl = (ObjClosure *)val_as_obj(fn);
    /* 建协程 context（小栈起始，弹性增长；挂 GC contexts 链） */
    mica_context *cctx = mica_context_new_sized(ctx->rt, MICA_CORO_STACK_INIT, false);
    cctx->coro_closure = cl;
    CTX_SET_CORO(cctx, CORO_SUSPENDED);
    if (argc > 1) {
        cctx->coro_start_args = malloc((size_t)(argc - 1) * sizeof(Value));
        assert(cctx->coro_start_args != NULL);
        cctx->n_coro_start_args = (int)(argc - 1);
        for (uint32_t i = 1; i < argc; i++) {
            cctx->coro_start_args[i - 1] = val_unwrap(args[i]);
            gc_record_write(ctx->gc, (Obj *)cctx, cctx->coro_start_args[i - 1]);
        }
    }
    results[0] = val_wrap(val_obj((Obj *)cctx));
    return 1;
}

/* ---------- coroutine.resume ---------- */

static uint32_t native_coroutine_resume(mica_context *ctx, mica_value *args, uint32_t argc,
                                        mica_value *results) {
    mica_context *cctx = coro_arg(ctx, args, argc, "coroutine.resume");
    if (!cctx) return 0;
    if (CTX_CORO_STATUS(cctx) == CORO_DEAD) {
        results[0] = val_wrap(make_error(ctx, "cannot resume dead coroutine"));
        return 1;
    }
    if (CTX_CORO_STATUS(cctx) == CORO_RUNNING) {
        results[0] = val_wrap(make_error(ctx, "cannot resume running coroutine"));
        return 1;
    }
    Value vargs[MICA_NATIVE_MAX_RESULTS];
    int nargs = (int)argc - 1;
    if (nargs > MICA_NATIVE_MAX_RESULTS) nargs = MICA_NATIVE_MAX_RESULTS;
    for (int i = 0; i < nargs; i++) vargs[i] = val_unwrap(args[1 + i]);
    Value out = val_null();
    const int r = vm_coroutine_run(ctx, cctx, vargs, nargs, &out);
    if (r == CORO_ERROR) {
        /* Phase 2g：把原错误对象（如有）作为错误值返回（Mica 错误即值；
           try/catch 在调用点捕获、未捕获即"抛出"崩溃）。 */
        if (!val_is_null(cctx->st_error_value)) results[0] = val_wrap(cctx->st_error_value);
        else results[0] = val_wrap(make_error(ctx, cctx->st_error_msg ? cctx->st_error_msg
                                                                  : "coroutine error"));
        return 1;
    }
    results[0] = val_wrap(out);
    return 1;
}

/* ---------- coroutine.cancel ---------- */

static uint32_t native_coroutine_cancel(mica_context *ctx, mica_value *args, uint32_t argc,
                                        mica_value *results) {
    mica_context *cctx = coro_arg(ctx, args, argc, "coroutine.cancel");
    if (!cctx) return 0;
    if (CTX_CORO_STATUS(cctx) == CORO_RUNNING || CTX_CORO_STATUS(cctx) == CORO_NORMAL) {
        runtime_error(ctx, NULL, "cannot cancel running coroutine");
        return 0;
    }
    vm_coroutine_cancel(ctx, cctx); /* DEAD 幂等：重复 cancel 无害 */
    results[0] = mica_null();
    return 1;
}

/* ---------- coroutine.status ---------- */

static uint32_t native_coroutine_status(mica_context *ctx, mica_value *args, uint32_t argc,
                                        mica_value *results) {
    mica_context *cctx = coro_arg(ctx, args, argc, "coroutine.status");
    if (!cctx) return 0;
    const char *s = "suspended";
    switch (CTX_CORO_STATUS(cctx)) {
        case CORO_RUNNING: s = "running"; break;
        case CORO_NORMAL:  s = "normal";  break;
        case CORO_DEAD:    s = "dead";    break;
        default: break;
    }
    results[0] = val_wrap(obj_string_val(obj_string_new(ctx->gc, s, strlen(s))));
    return 1;
}

/* ---------- 协程 C API（Phase 2e：宿主驱动协程） ---------- */

mica_value mica_coroutine_create(mica_context *ctx, mica_value fn) {
    mica_value args[1] = { fn };
    mica_value results[1];
    const uint32_t n = native_coroutine_create(ctx, args, 1, results);
    return n ? results[0] : mica_null();
}

int mica_resume(mica_context *ctx, mica_value co, const mica_value *args, int argc,
                mica_value *result) {
    mica_context *cctx = coro_arg(ctx, (mica_value *)(void *)&co, 1, "mica_resume");
    if (!cctx) return MICA_RESUME_ERROR;
    if (CTX_CORO_STATUS(cctx) == CORO_DEAD || CTX_CORO_STATUS(cctx) == CORO_RUNNING) return MICA_RESUME_ERROR;
    Value vargs[MICA_NATIVE_MAX_RESULTS];
    int nargs = argc;
    if (nargs > MICA_NATIVE_MAX_RESULTS) nargs = MICA_NATIVE_MAX_RESULTS;
    for (int i = 0; i < nargs; i++) vargs[i] = val_unwrap(args[i]);
    Value out = val_null();
    const int r = vm_coroutine_run(ctx, cctx, vargs, nargs, &out);
    if (r == CORO_ERROR) return MICA_RESUME_ERROR; /* 错误在 cctx（宿主读 ctx->... 无；读 cctx 内部） */
    if (result) *result = val_wrap(out);
    return (r == CORO_YIELDED) ? MICA_RESUME_YIELD : MICA_RESUME_DONE;
}

int mica_coroutine_status(mica_context *ctx, mica_value co) {
    mica_context *cctx = coro_arg(ctx, (mica_value *)(void *)&co, 1, "mica_coroutine_status");
    if (!cctx) return MICA_CORO_DEAD;
    return (int)CTX_CORO_STATUS(cctx);
}

int mica_coroutine_cancel(mica_context *ctx, mica_value co) {
    mica_context *cctx = coro_arg(ctx, (mica_value *)(void *)&co, 1, "mica_coroutine_cancel");
    if (!cctx) return 1;
    if (CTX_CORO_STATUS(cctx) == CORO_RUNNING || CTX_CORO_STATUS(cctx) == CORO_NORMAL) return 1;
    vm_coroutine_cancel(ctx, cctx);
    return 0;
}

/* ---------- 宿主 native 挂起协程（Phase 2c） ----------
   mica_yield：native 置 yield_requested + 产出值 + 续体，正常 return；
   解释器在 native 调用点检测标志 → 复用 CORO_YIELDED 挂起路径（无 longjmp）。
   mica_callk：调 Mica 闭包，内层 yield 时自动注册 k+ud 为续体并透传产出（返回
   MICA_CALL_YIELDED）；native 只需 return 0。 */

void mica_yield(mica_context *ctx, mica_value yield_value, mica_continuation cont, void *ud) {
    ctx->yield_requested = true;
    /* 产出值写入 context 字段 = 引用型写入 old 容器：须登记写屏障（context 可能已是
       old/黑），否则 young 产出值只在 yield_value 里而无 gray 登记 → minor 误回收。
       同时 context_trace 已把 yield_value 纳入根集（双保险）。 */
    ctx->yield_value = val_unwrap(yield_value);
    gc_record_write(ctx->gc, (Obj *)ctx, ctx->yield_value);
    ctx->yield_cont = cont;
    ctx->yield_ud = ud;
}

int mica_callk(mica_context *ctx, mica_value fn, mica_value *args, uint32_t argc,
               mica_continuation k, void *ud, mica_value *results) {
    Value f = val_unwrap(fn);
    if (!val_is_obj(f) || val_as_obj(f)->type != OBJ_CLOSURE) {
        runtime_error(ctx, ctx_frame(ctx), "mica_callk expects a function");
        return -2;
    }
    ObjClosure *cl = (ObjClosure *)val_as_obj(f);
    Proto *fp = cl->proto;
    Value av[MICA_NATIVE_MAX_RESULTS];
    uint32_t an = argc > MICA_NATIVE_MAX_RESULTS ? MICA_NATIVE_MAX_RESULTS : argc;
    for (uint32_t i = 0; i < an; i++) av[i] = val_unwrap(args[i]);

    CallFrame *top = ctx_frame(ctx);
    Value *rbase = NULL;
    int stop;
    if (top->closure == cl) {
        /* 内层 yield 后恢复：top 帧就是被挂起的 callee（fn 相同）→ 续跑（写恢复参数） */
        const Instr yin = top->savedpc[-1];
        top->base[instr_a(yin)] = (an > 0) ? av[0] : val_null();
        rbase = top->base;
        stop = ctx->frame_count - 1; /* 跑到 callee 返回（弹帧到 native 帧） */
    } else {
        /* fresh call：在当前帧寄存器顶之上压 callee 帧（同 call_overload/vm_call_value） */
        Value *base = top->base + (int)top->closure->proto->max_regs;
        if (!vm_ensure_stack(ctx, base + 1 + (int)fp->max_regs)) {
            runtime_error(ctx, top, "stack overflow (mica_callk)");
            return -2;
        }
        base = top->base + (int)top->closure->proto->max_regs;
        if (ctx->frame_count >= MICA_FRAME_MAX_SOFT) {
            runtime_error(ctx, top, "too many frames (mica_callk)");
            return -2;
        }
        Value *argbuf = an ? malloc((size_t)an * sizeof(Value)) : NULL;
        if (an && !argbuf) return -2;
        for (uint32_t i = 0; i < an; i++) argbuf[i] = av[i];
        *base = obj_closure_val(cl);
        base[1] = val_null(); /* 统一约定：slot1=null self */
        Value *slot = base + 2;
        for (uint32_t i = 0; i < an; i++) *slot++ = argbuf[i];
        free(argbuf);
        while ((slot - base) <= (int)fp->arity) *slot++ = val_null();
        CallFrame *nf = ctx_frame_new(ctx);
        *nf = (CallFrame){ .closure = cl, .savedpc = fp->code, .u = { 0 }, .base = base,
                           .want = 1, .defer_mark = ctx->defer_top };
        nf->argc = (uint16_t)(an + 1); /* vararg：实参数（含 self 槽） */
        ctx->stack_top = base + (int)fp->max_regs;
        rbase = base;
        stop = ctx->frame_count - 1; /* 当前帧数（callee 已压）：跑到 callee 返回 */
    }
    Value yval = val_null();
    const CoroResult r = vm_run(ctx, &yval, stop);
    /* 恢复 native 帧栈顶（callee 已跑完/挂起） */
    CallFrame *nat = ctx_frame(ctx);
    ctx->stack_top = nat->base + (int)nat->closure->proto->max_regs;
    if (r == CORO_YIELDED) {
        /* 内层 yield：注册续体 + 透传产出 */
        ctx->yield_cont = k;
        ctx->yield_ud = ud;
        ctx->yield_requested = true;
        ctx->yield_value = yval;
        gc_record_write(ctx->gc, (Obj *)ctx, ctx->yield_value); /* 同 mica_yield：须屏障 */
        return MICA_CALL_YIELDED;
    }
    if (r == CORO_ERROR) {
        vm_rethrow(ctx); /* 错误上抛到调用方（外层 vm_run / 标志式返回） */
        return -2;
    }
    results[0] = val_wrap(rbase[0]); /* callee 返回值在 base[0]（slot0=函数值） */
    return 1;
}

/* ---------- 注册 ---------- */

void libcoro_init(mica_runtime *rt) {
    mica_register_module(rt, "coroutine");
    mica_register_function(rt, "coroutine", "create", native_coroutine_create, 1);
    mica_register_function(rt, "coroutine", "resume", native_coroutine_resume, 1);
    mica_register_function(rt, "coroutine", "cancel", native_coroutine_cancel, 1);
    mica_register_function(rt, "coroutine", "status", native_coroutine_status, 1);
}
