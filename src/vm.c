/* 寄存器 VM 执行引擎（独立模块，2026-08-01 从 mica.c 拆出）。
   - run_state → vm_run；所有 op 处理器、defer/协程辅助、值字符串化在此。
   - 只依赖 mica.h + vm.h + state.h（mica_context 定义）+ 内部模块头；
     VM 是纯函数，直接操作 mica_context 字段（栈/帧/defer）。
   - runtime_error/make_error 实现在 mica.c（builtin.c 与 VM 共用）。 */

#include "mica.h"
#include "vm.h"
#include "state.h"
#include "str.h"

#include <assert.h>
#include <intrin.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "obj.h"
#include "proto.h"
#include "gc.h"
#include "table.h"
#include "value.h"
#include "builtin.h"
#include "strlit.h"

/* 宿主 native（mica_native_fn）结果上限：staging 缓冲容量（旧 user_native_bridge 语义）。 */
#define MICA_NATIVE_MAX_RESULTS 8

/* 属性 getter/setter 帧基址辅助：见定义处（压父帧寄存器高水位之上，防 getter 帧重叠
   父帧存活寄存器——旧实现压 `fr->base+dest` 会在 dest 为低槽局部时把父帧 self 清成 null）。 */
static Value* prop_frame_base(mica_context* ctx, CallFrame* fr);


static char* dup_str(const char* s) {
    size_t n = strlen(s);
    char* p = malloc(n + 1);
    assert(p != NULL);
    memcpy(p, s, n + 1);
    return p;
}

/* 标识符池名 → 用户可读显示（@X → System.X 展开，错误消息用）。非 "@" 开头原样返回原指针。 */
static const char* ident_display(const String* name, char* buf, size_t cap) {
    const char* s = str_data(name);
    const size_t len = str_len(name);
    if (len >= 1 && s[0] == '@' && len + 7 <= cap) {
        const size_t n = str_expand_system(s, len, buf);
        buf[n] = '\0';
        return buf;
    }
    return s;
}

/* 常量下标 → ObjString*；常量是 null（缺省 set/case）或越界时返回 NULL。
   错误/枚举匹配的"缺省"由 codegen 发真实 null 常量（无哨兵槽），故无条件读 constants[idx]。 */
static ObjString* const_str_or_null(const Proto* ch, int idx) {
    if (idx < 0 || idx >= (int)ch->n_constants) return NULL;
    const Value v = ch->constants[idx];
    return (val_is_string(v)) ? (ObjString *)val_as_obj(v) : NULL;
}

/* Lua 式动态栈：需要栈顶指针 need_top（= stack + 已用槽数）时若不够则翻倍扩容。
   返回 false=超硬上限（调用方报栈溢出）。
   扩容后：新槽清零（未用槽可被 GC 安全扫描）；按偏移修正所有栈内指针——开放捕获值 p、
   全部帧的 base / getter_base、ctx->stack_top（用户要求：栈增长时遍历更新）。 */
static bool ensure_stack(mica_context* ctx, const Value* need_top) {
    if (MICA_LIKELY(need_top <= ctx->stack + ctx->stack_size)) return true;
    const int old_size = ctx->stack_size;
    const ptrdiff_t need = need_top - ctx->stack; /* 需要的槽数 */
    size_t nsize = old_size ? (size_t)old_size * 2 : MICA_STACK_SIZE;
    while (nsize < (size_t)need) nsize *= 2;
    if (nsize > MICA_STACK_MAX) return false;
    Value* ns = realloc(ctx->stack, nsize * sizeof(Value));
    if (!ns) return false;
    memset(ns + old_size, 0, (nsize - (size_t)old_size) * sizeof(Value));
    const ptrdiff_t delta = ns - ctx->stack; /* 单位：Value 数 */
    /* 开放捕获值 p：仅 open（未 closed）的指向栈 */
    for (ObjCapVal* cap = ctx->opencapval; cap; cap = cap->open_next) {
        if (!(cap->flags & OBJ_CAPVAL_CLOSED)) cap->p += delta;
    }
    /* 所有帧的 base + getter_base（getter 帧才读 getter_base）；帧块 pointer-stable，
       fr 指针本身无需平移，仅栈内指针按 delta 更新 */
    for (FrameBlock* b = ctx->frame_blocks; b; b = b->next) {
        for (int i = 0; i < b->used; i++) {
            CallFrame* f = &b->frames[i];
            f->base += delta;
            if (f->flags & CF_GETTER) f->u.getter.getter_base += delta;
        }
    }
    ctx->stack_top += delta;
    ctx->stack = ns;
    ctx->stack_size = (int)nsize;
    return true;
}

/* 内部工具导出（libcoro 等扩展模块用）：ensure_stack（vm_rethrow 定义在 vm_rethrow_error 后）。 */
bool vm_ensure_stack(mica_context* ctx, const Value* need_top) {
    return ensure_stack(ctx, need_top);
}


/* 数组下标规整 + 边界检查（所有 array 快速路径统一走此函数）：
   - 非整数（如 1.5 / -0.5）**必须报错**，不能静默截断（旧实现 `(int64_t)` 直接把
     arr[1.5] 读成 arr[1]、arr[-0.5] 读成 arr[0] → 静默错值；Lua 5.4 亦拒绝）。
   - 越界报错（带可读的长度）。返回 false 表示已报错（调用方直接 VM_DISPATCH）。
   成功时 *out 是合法下标。 */
MICA_FORCEINLINE bool array_index_of(mica_context* ctx, CallFrame* fr, const Instr* pc,
                                     const ObjArray* a, Value idx, int64_t* out) {
    const double d = val_as_float(idx);
    if (MICA_UNLIKELY(d != (double)(int64_t)d)) {
        runtime_error_at(ctx, fr, pc, "array index must be an integer (got %g)", d);
        return false;
    }
    const int64_t i = (int64_t)d;
    if (MICA_UNLIKELY((uint64_t)i >= a->count)) {
        runtime_error_at(ctx, fr, pc, "index %" PRId64 " out of bounds (len %u)", i, a->count);
        return false;
    }
    *out = i;
    return true;
}

/* ---------- defer 运行辅助 ---------- */

static void defer_push(mica_context* ctx, Value v) {
    if (ctx->defer_top >= ctx->defer_cap) { /* 变长：容量翻倍 */
        size_t ncap = ctx->defer_cap ? (size_t)ctx->defer_cap * 2 : MICA_DEFER_STACK;
        ctx->defers = realloc(ctx->defers, ncap * sizeof(Value));
        assert(ctx->defers != NULL);
        ctx->defer_cap = (int)ncap;
    }
    ctx->defers[ctx->defer_top++] = v;
    gc_record_write(ctx->gc, (Obj *)ctx, v); /* 老 context 的 defer 新引用须登记（minor 重扫） */
}

/* 从 defer 栈弹出到帧起始 mark 之间的闭包（遇块标记停止），LIFO 顺序放入 out。 */
static size_t defer_collect(mica_context* ctx, int mark, Value* out, size_t cap) {
    size_t n = 0;
    while (ctx->defer_top > mark) {
        Value d = ctx->defers[--ctx->defer_top];
        if (val_is_null(d)) break; /* 块标记 */
        if (n < cap) out[n++] = d;
    }
    return n;
}

/* 依次运行 defer 闭包（嵌套 vm_run；defer 内 yield 视为错误）。
   ⚠️ 调用方在 defer_collect 里已把闭包**弹出**被 trace 的 ctx->defers 到 C 数组，
   C 数组不受 GC 管辖；defer_run_closures 内每个闭包的函数体都可能分配并触发 minor，
   若不设 unsafe 窗口，#1..#n-1 只被 C 数组引用 → 被误回收（用后释放）。
   stop_count 参数用于嵌套 vm_run 的停止点，不影响本窗口。 */
static void defer_run_closures(mica_context* ctx, const Value* closures, size_t n, int stop_count) {
    /* ⚠️ runtime_error 在 vm_run 内会 longjmp（active_jmp 有效）→ 它之后的 gc_end_unsafe
       永远执行不到，unsafe_depth 会永久 >0 从而**彻底关掉 GC**。故所有失败点先把窗口
       收掉再报错，或用统一出口。 */
    gc_begin_unsafe(ctx->gc);
    for (size_t i = 0; i < n && !CTX_HAS_ERROR(ctx); i++) {
        Value d = closures[i];
        if (!(val_is_closure(d))) {
            gc_end_unsafe(ctx->gc);
            runtime_error(ctx, NULL, "invalid defer value");
            return;
        }
        ObjClosure* cl = (ObjClosure *)val_as_obj(d);
        Proto* f = cl->proto;
        /* 用偏移而非裸指针跨越 ensure_stack / 嵌套 vm_run（栈可能 realloc） */
        const ptrdiff_t base_off = (ptrdiff_t)(ctx->stack_top - ctx->stack);
        if (!ensure_stack(ctx, ctx->stack + base_off + (int)f->max_regs)) {
            gc_end_unsafe(ctx->gc);
            runtime_error(ctx, NULL, "stack overflow in defer");
            return;
        }
        if (ctx->frame_count >= MICA_FRAME_MAX_SOFT) {
            gc_end_unsafe(ctx->gc);
            runtime_error(ctx, NULL, "too many frames in defer");
            return;
        }
        Value* base = ctx->stack + base_off; /* ensure_stack realloc 后重新求值（指针） */
        *base = obj_closure_val(cl);
        ctx->stack_top = base + (int)f->max_regs;
        CallFrame *df = ctx_frame_new(ctx);
        *df = (CallFrame){
                    .closure = cl, .savedpc = f->code, .u = { 0 }, .base = base,
                    .flags = CF_DEFER, .want = RESULT_DISCARD, .defer_mark = ctx->defer_top,
                };
        Value ignored;
        const CoroResult r = vm_run(ctx, &ignored, stop_count);
        if (r == CORO_YIELDED) {
            gc_end_unsafe(ctx->gc);
            runtime_error(ctx, NULL, "defer cannot yield");
            return;
        }
        ctx->stack_top = ctx->stack + base_off; /* 嵌套 vm_run 可能 realloc，按偏移回写 */
    }
    gc_end_unsafe(ctx->gc);
}

/* ============================================================
   协程引擎（Phase 1.5+：协程实例 = mica_context，协程值 = OBJ_CONTEXT）。
   语言层 API 在 libcoro（coroutine.create/resume/cancel/status native）；
   本文件提供底层引擎供 libcoro / mica_resume（C API）调用。
   ============================================================ */

#define MICA_RESUME_MAX_DEPTH 200 /* 嵌套 resume 深度护栏（对齐 LUAI_MAXCCALLS=200） */

/* 取消协程（Lua close 语义，Phase 2a）：跑 pending defer + 逐帧关 capval + 释放帧块，
   状态置 DEAD。之后 resume 报错。defer 运行期错误 → CTX_HAS_ERROR(cctx)（调用方处理）。 */
void vm_coroutine_cancel(mica_context* ctx, mica_context* cctx) {
    (void)ctx;
    CTX_SET_CORO(cctx, CORO_DEAD);
    const int mark = (cctx->frame_count > 0) ? ctx_frame_first(cctx)->defer_mark : 0;
    size_t count = 0;
    for (int i = cctx->defer_top - 1; i >= mark; i--) {
        if (!val_is_null(cctx->defers[i])) count++;
    }
    Value* defers = malloc((count ? count : 1) * sizeof(Value));
    size_t n = 0;
    while (cctx->defer_top > mark) {
        Value d = cctx->defers[--cctx->defer_top];
        if (!val_is_null(d)) defers[n++] = d;
    }
    /* 协程终结：关闭所有帧的开放捕获值，避免闭包持指向将释放栈的指针 */
    for (FrameBlock* b = cctx->frame_blocks; b; b = b->next) {
        for (int i = 0; i < b->used; i++) {
            obj_frame_close_capvals(cctx->gc, cctx, &b->frames[i]);
        }
        b->used = 0;
    }
    cctx->frame_count = 0;
    cctx->stack_top = cctx->stack;
    CTX_CLR_ERROR(cctx);
    defer_run_closures(cctx, defers, n, 0);
    free(defers);
    /* 确定性释放执行态内存：帧块链整体 free + 栈缩回初始小栈（dead 后不再 resume） */
    ctx_frame_blocks_free(cctx);
    if (cctx->stack_size > MICA_CORO_STACK_INIT) {
        Value* ns = realloc(cctx->stack, MICA_CORO_STACK_INIT * sizeof(Value));
        if (ns) {
            cctx->stack = ns;
            cctx->stack_size = MICA_CORO_STACK_INIT;
        }
    }
    cctx->stack_top = cctx->stack;
}

/* 协程 resume 引擎（供 libcoro / mica_resume 调用）：
   - 状态检查（DEAD/RUNNING/深度护栏）+ 装载启动/恢复参数 + vm_run(cctx)。
   - 返回 CORO_YIELDED（*out=产出值）/ CORO_DONE（*out=完成值）/ CORO_ERROR
     （cctx->st_error_msg/st_error_value 有效；调用方决定 rethrow 或返回值）。
   - 状态管理：caller（ctx）是协程时置 NORMAL，cctx 置 RUNNING，结束后恢复。 */
int vm_coroutine_run(mica_context* ctx, mica_context* cctx, const Value* args, int nargs,
                     Value* out) {
    if (cctx->resume_depth >= MICA_RESUME_MAX_DEPTH) {
        ctx_set_error(cctx, "coroutine resume too deep");
        return CORO_ERROR;
    }
    /* 重入防护：只能 resume SUSPENDED（或尚未启动）的协程。若允许 resume 处于
       RUNNING/NORMAL 的协程（A resume B、B 又 resume A 的互递归），会**在同一个
       context 上启动第二个并发 vm_run 激活**——两者共享 stack_top/cur_block/active_jmp，
       而外层激活仍用缓存的 R/pc 继续跑 → 状态互相踩踏（实测 A/B 互 resume 后返回值错位、
       yield 值投递到错误的 resume）。语言层 native（libcoro）与 C API 已有此检查，这里补齐。 */
    {
        const int st = CTX_CORO_STATUS(cctx);
        if (st == CORO_RUNNING || st == CORO_NORMAL) {
            ctx_set_error(cctx, "cannot resume a %s coroutine",
                          st == CORO_RUNNING ? "running" : "normal");
            return CORO_ERROR;
        }
    }
    const bool caller_is_coro = CTX_IS_CORO(ctx);
    if (caller_is_coro) CTX_SET_CORO(ctx, CORO_NORMAL);
    CTX_SET_CORO(cctx, CORO_RUNNING);
    cctx->resume_depth++;
    bool cont_yielded = false;   /* 续体内部又挂起（mica_callk 内层 yield） */
    Value cont_yval = val_null();
    if (cctx->frame_count == 0) {   /* 未启动（frame_count==0 且非 DEAD）：首 resume 建启动帧 */
        /* ⚠️ 协程栈起始小（MICA_CORO_STACK_INIT=128），必须 ensure_stack 到 max_regs，
           否则写越界 → 堆损坏（老模型 16384 栈掩盖此坑）。 */
        if (!ensure_stack(cctx, cctx->stack + (int)cctx->coro_closure->proto->max_regs)) {
            ctx_set_error(cctx, "stack overflow (coroutine start)");
            cctx->resume_depth--;
            CTX_SET_CORO(cctx, CORO_DEAD);
            if (caller_is_coro) CTX_SET_CORO(ctx, CORO_RUNNING);
            return CORO_ERROR;
        }
        cctx->stack[0] = obj_closure_val(cctx->coro_closure);
        cctx->stack[1] = val_null(); /* 统一约定：slot1=null self（协程体普通函数） */
        int na = nargs;
        if (cctx->n_coro_start_args > 0) {
            for (int i = 0; i < cctx->n_coro_start_args; i++)
                cctx->stack[2 + i] = cctx->coro_start_args[i];
            na = cctx->n_coro_start_args;
            free(cctx->coro_start_args);
            cctx->coro_start_args = NULL;
            cctx->n_coro_start_args = 0;
        } else {
            for (int i = 0; i < nargs; i++) cctx->stack[2 + i] = args[i];
        }
        for (int i = 2 + na; i < (int)cctx->coro_closure->proto->arity + 1; i++)
            cctx->stack[i] = val_null();
        cctx->stack_top = cctx->stack + (int)cctx->coro_closure->proto->max_regs;
        cctx->defer_top = 0;
        CallFrame *cf0 = ctx_frame_new(cctx);
        *cf0 = (CallFrame){
            .closure = cctx->coro_closure, .savedpc = cctx->coro_closure->proto->code,
            .u = { 0 }, .base = cctx->stack, .want = 1,
        };
        cf0->argc = (uint16_t)(na + 1); /* vararg：启动实参数（含 self 槽） */
    } else if (cctx->yield_cont) {
        /* Phase 2c：native 挂起（mica_yield/mica_callk）→ resume 调续体，结果写回
           native 调用结果槽（yield_res_reg），随后从 savedpc（CALL 之后）继续。
           若续体内部又挂起（mica_callk 内层 yield）：cctx->yield_requested 被置位，
           本次 resume 仍为挂起（产出续体透传的新值）。 */
        mica_continuation cont = cctx->yield_cont;
        void *ud = cctx->yield_ud;
        cctx->yield_cont = NULL;
        cctx->yield_ud = NULL;
        mica_value cargs[MICA_NATIVE_MAX_RESULTS];
        int cn_args = nargs;
        if (cn_args > MICA_NATIVE_MAX_RESULTS) cn_args = MICA_NATIVE_MAX_RESULTS;
        for (int i = 0; i < cn_args; i++) cargs[i] = val_wrap(args[i]);
        mica_value cres[MICA_NATIVE_MAX_RESULTS];
        const uint32_t cn = cont(cctx, ud, cargs, (uint32_t)cn_args, cres);
        if (cctx->yield_requested) {
            cctx->yield_requested = false;
            cont_yielded = true;
            cont_yval = cctx->yield_value;
        } else {
            CallFrame *yf = ctx_frame(cctx);
            yf->base[cctx->yield_res_reg] = (cn > 0) ? val_unwrap(cres[0]) : val_null();
        }
    } else {
        /* yield 恢复：把 resume 实参写入 yield 指令的 a 寄存器（savedpc[-1] 是 YIELD） */
        CallFrame* yf = ctx_frame(cctx);
        const Instr yin = yf->savedpc[-1];
        yf->base[instr_a(yin)] = (nargs > 0) ? args[0] : val_null();
    }
    if (cont_yielded) {
        cctx->resume_depth--;
        CTX_SET_CORO(cctx, CORO_SUSPENDED);
        if (caller_is_coro) CTX_SET_CORO(ctx, CORO_RUNNING);
        *out = cont_yval;
        return CORO_YIELDED;
    }
    Value result = val_null();
    CoroResult r;
    {
        r = vm_run(cctx, &result, 0);
    }
    cctx->resume_depth--;
    CTX_SET_CORO(cctx, (r == CORO_YIELDED) ? CORO_SUSPENDED : CORO_DEAD);
    if (caller_is_coro) CTX_SET_CORO(ctx, CORO_RUNNING);
    *out = result;
    return (int)r;
}

/* OP_CLOSURE：为当前帧的局部槽建/find-or-create 开放捕获值。
   同一栈槽被多个 lambda 捕获时共享同一捕获值（保证读到的变量一致）。
   find-or-create 遍历 context.openupval 链（Lua luaF_findupval 风格）：slot 地址唯一，
   跨帧不冲突；帧弹出时 obj_frame_close_capvals 按地址范围关闭。 */
MICA_FORCEINLINE ObjCapVal* frame_capval_for(mica_context* ctx, CallFrame* fr, int reg_index) {
    Value* slot = fr->base + reg_index;
    for (ObjCapVal* cap = ctx->opencapval; cap; cap = cap->open_next) {
        if (cap->p == slot) return cap;
    }
    ObjCapVal* cap = obj_capval_new(ctx->gc, val_null());
    cap->p = slot;
    cap->flags &= ~OBJ_CAPVAL_CLOSED; /* open：清 closed 标志（union 下 p 指向栈槽） */
    cap->open_next = ctx->opencapval; /* open：头插 context 的 open 链（Lua openupval 等价） */
    cap->open_prev = NULL;
    if (ctx->opencapval) ctx->opencapval->open_prev = cap;
    ctx->opencapval = cap;
    return cap;
}

char* value_to_cstr(Value v) {
    if (val_is_string(v)) {
        ObjString* s = (ObjString *)val_as_obj(v);
        char* p = malloc(s->len + 1);
        assert(p != NULL);
        memcpy(p, s->chars, s->len);
        p[s->len] = '\0';
        return p;
    }
    if (val_is_obj(v) && val_as_obj(v)->type == OBJ_ERROR) {
        ObjString* msg = ((ObjError *)val_as_obj(v))->message;
        char* p = malloc(msg->len + 1);
        assert(p != NULL);
        memcpy(p, msg->chars, msg->len);
        p[msg->len] = '\0';
        return p;
    }
    if (val_is_instance(v)) {
        ObjClass* k = ((ObjInstance *)val_as_obj(v))->klass;
        const char* n = k->name ? k->name->chars : "?";
        size_t nlen = strlen(n);
        char* p = malloc(nlen + 3);
        assert(p != NULL);
        p[0] = '<';
        memcpy(p + 1, n, nlen);
        p[nlen + 1] = '>';
        p[nlen + 2] = '\0';
        return p;
    }
    if (val_is_float(v)) {
        char buf[48];
        const size_t n = value_num_to_buf(val_as_float(v), buf, sizeof buf);
        if (n) {
            char *p = malloc(n + 1);
            assert(p != NULL);
            memcpy(p, buf, n);
            p[n] = '\0';
            return p;
        }
        snprintf(buf, sizeof(buf), "%.17g", val_as_float(v));
        return dup_str(buf);
    }
    switch (val_tag(v)) {
    case MICA_TNULL: return dup_str("null");
    case MICA_TFALSE:
    case MICA_TTRUE: return dup_str(val_as_bool(v) ? "true" : "false");
    case MICA_TSTR:
    case MICA_TARRAY:
    case MICA_TMAP:
    case MICA_TINSTANCE:
    case MICA_TCLOSURE:
    case MICA_TOBJ: { /* array/map 用打印 */
        FILE* f = tmpfile();
        if (!f) return dup_str("<obj>");
        value_print(f, v);
        long sz = ftell(f);
        if (sz < 0) sz = 0;
        rewind(f);
        char* p = malloc((size_t)sz + 1);
        assert(p != NULL);
        size_t rd = fread(p, 1, (size_t)sz, f);
        p[rd] = '\0';
        fclose(f);
        return p;
    }
    default:
        return dup_str("?");
    }
}

/* ---------- 类运算符重载 ---------- */

/* 运算符重载方法名字面量（与 state.h OP_NAME_* 索引一一对应）。 */
static const char* const k_op_name_lits[OP_NAME_COUNT] = {
    "__add__", "__sub__", "__mul__", "__div__", "__idiv__",
    "__mod__", "__pow__", "__neg__", "__lt__", "__le__",
    "__gt__", "__ge__", "__eq__", "__ne__", "__index__",
    "__index_set__",
};

/* 取运算符重载方法名（permanent ObjString，惰性创建缓存到 runtime——
   避免每次 call_overload 都 obj_string_new 分配查找键，减 GC 压力）。 */
MICA_FORCEINLINE ObjString* op_name(mica_context* ctx, int idx) {
    ObjString* s = ctx->rt->op_names[idx];
    if (!s) {
        s = obj_string_new_permanent(ctx->gc, k_op_name_lits[idx], strlen(k_op_name_lits[idx]));
        ctx->rt->op_names[idx] = s;
    }
    return s;
}

/* 错误重新抛出：嵌套 vm_run（重载/反射/defer/属性 getter-setter）捕获错误后置 has_error 返回；
   若外层有活跃 vm_run（active_jmp 有效）则 longjmp 展开让错误立即冒泡到最近 setjmp，
   否则（mica_call 直调 native 无外层 vm_run）保持标志式返回。 */
MICA_FORCEINLINE void vm_rethrow_error(mica_context* ctx) {
    if (ctx->active_jmp) longjmp(*ctx->active_jmp, 1);
}

/* 内部工具导出（libcoro 等扩展模块用）：错误重抛。 */
void vm_rethrow(mica_context* ctx) {
    vm_rethrow_error(ctx);
}

/* 调用实例的运算符重载方法（__add__/__lt__/__index__ 等）。
   用嵌套 vm_run 运行：栈顶开新帧，self=receiver，后接 nargs 个实参。
   返回 true=已处理（含出错，错误已传播）；false=实例无此方法（调用方走默认）。 */
MICA_FORCEINLINE bool call_overload(mica_context* ctx, CallFrame* fr, Value receiver, int name_idx,
                                    const Value* args, int nargs, Value* out) {
    if (!(val_is_instance(receiver))) return false;
    ObjInstance* inst = (ObjInstance *)val_as_obj(receiver);
    ObjClosure* m = obj_class_find_method(inst->klass, op_name(ctx, name_idx));
    if (!m) return false;
    const ptrdiff_t saved_top = ctx->stack_top - ctx->stack; /* 相对偏移：ensure_stack 可能移动栈 */
    /* 用当前帧的寄存器顶之上作嵌套方法基址：ctx->stack_top 在调用后会变低，
       可能低于当前语句的临时寄存器，导致嵌套方法覆盖它们。 */
    Value* base = fr->base + (int)fr->closure->proto->max_regs;
    if (!ensure_stack(ctx, base + 1 + (int)m->proto->max_regs)) {
        runtime_error(ctx, fr, "stack overflow (operator overload '%s')", k_op_name_lits[name_idx]);
        *out = val_null();
        return true;
    }
    base = fr->base + (int)fr->closure->proto->max_regs; /* ensure_stack realloc 后重算（指针） */
    if (ctx->frame_count >= MICA_FRAME_MAX_SOFT) {
        runtime_error(ctx, fr, "too many frames (operator overload '%s')", k_op_name_lits[name_idx]);
        *out = val_null();
        return true;
    }
    const int stop = ctx->frame_count;
    *base = obj_closure_val(m);
    base[1] = receiver;                 /* self */
    for (int i = 0; i < nargs; i++) base[2 + i] = args[i];
    for (int i = 1 + nargs; i < (int)m->proto->arity; i++) base[1 + i] = val_null();
    CallFrame *nf = ctx_frame_new(ctx);
    *nf = (CallFrame){ .closure = m, .savedpc = m->proto->code, .u = { 0 }, .base = base,
                       .want = 1, .defer_mark = ctx->defer_top };
    ctx->stack_top = base + (int)m->proto->max_regs;
    /* 嵌套运行：vm_run 的 *out 只在 stop=0（帧数归零）时有效；
       这里方法帧被 OP_RETURN 弹出后，返回值在其 base 槽（stack[base]）。 */
    const CoroResult r = vm_run(ctx, NULL, stop);
    ctx->stack_top = ctx->stack + saved_top;
    if (r == CORO_ERROR) {
        vm_rethrow_error(ctx); /* 外层 vm_run 内：longjmp 展开（stack.top 已恢复）；否则标志式返回 */
        *out = val_null(); /* 错误已在状态中 */
        return true;
    }
    if (r == CORO_YIELDED) {
        runtime_error(ctx, fr, "operator overload method cannot yield");
        *out = val_null();
        return true;
    }
    /* 嵌套 vm_run 内可能 ensure_stack realloc 栈 → base 悬垂，必须按偏移重算 */
    *out = ctx->stack[(ptrdiff_t)(base - ctx->stack)];
    return true;
}

/* 从 native/builtin 上下文调用函数值（ObjClosure/ObjNativeClosure）。
   嵌套帧建在当前帧寄存器顶之上（fr->base + fr->closure->proto->max_regs），不干扰调用方寄存器。
   返回 true=已处理（含错误，错误时 *out=null 且 ctx 错误已置）；
   返回 false=不可调用的值（错误已置）。 */
bool vm_call_value(mica_context* ctx, CallFrame* fr, Value fn, Value self, const Value* args,
                   int nargs, Value* out) {
    if (!fr) {
        runtime_error(ctx, NULL, "cannot invoke a function outside a frame");
        return false;
    }
    Proto* f = NULL;
    ObjClosure* cl = NULL;
    if (MICA_LIKELY(val_is_closure(fn))) {
        cl = (ObjClosure *)val_as_obj(fn);
        f = cl->proto;
    } else if (val_is_obj(fn) && val_as_obj(fn)->type == OBJ_NATIVE_CLOSURE) {
        ObjNativeClosure* nat = (ObjNativeClosure *)val_as_obj(fn);
        mica_value nres[MICA_NATIVE_MAX_RESULTS];
        /* 统一约定：方法 native receiver 作首参（self + args）；普通 native 纯实参（args） */
        Value sbuf[MICA_NATIVE_MAX_RESULTS + 1];
        const Value* aptr;
        uint32_t an;
        if (nat->is_method) {
            sbuf[0] = self;
            for (int i = 0; i < nargs && i < MICA_NATIVE_MAX_RESULTS; i++) sbuf[1 + i] = args[i];
            aptr = sbuf;
            an = (uint32_t)nargs + 1;
        } else {
            aptr = args;
            an = (uint32_t)nargs;
        }
        const uint32_t n = nat->func(ctx, (mica_value *)(void *)aptr, an, nres);
        if (n == 0) *out = val_null();
        else *out = val_unwrap(nres[0]);
        return true;
    } else {
        runtime_error(ctx, fr, "cannot invoke %s", value_type_name(fn));
        return false;
    }
    Value* base = fr->base + (int)fr->closure->proto->max_regs;
    if (!ensure_stack(ctx, base + 1 + (int)f->max_regs)) {
        runtime_error(ctx, fr, "stack overflow (reflection invoke)");
        *out = val_null();
        return true;
    }
    base = fr->base + (int)fr->closure->proto->max_regs; /* ensure_stack realloc 后重算（指针） */
    if (ctx->frame_count >= MICA_FRAME_MAX_SOFT) {
        runtime_error(ctx, fr, "too many frames (reflection invoke)");
        *out = val_null();
        return true;
    }
    /* 先快照实参：嵌套帧可能压在调用方帧顶（base=max_regs 处），直接写嵌套帧会覆盖
       args 指针指向的调用方寄存器。 */
    Value* argbuf = nargs > 0 ? malloc((size_t)nargs * sizeof(Value)) : NULL;
    if (nargs > 0 && !argbuf) {
        runtime_error(ctx, fr, "out of memory (reflection invoke)");
        *out = val_null();
        return true;
    }
    for (int i = 0; i < nargs; i++) argbuf[i] = args[i];
    const ptrdiff_t saved_top = ctx->stack_top - ctx->stack; /* 相对偏移：ensure_stack 可能移动栈 */
    const int stop = ctx->frame_count;
    *base = obj_closure_val(cl);
    base[1] = self; /* 统一约定：slot1=self（普通函数 null） */
    Value* slot = base + 2;
    for (int i = 0; i < nargs; i++) *slot++ = argbuf[i]; /* 实参从 slot2 */
    free(argbuf);
    while ((slot - base) <= (int)f->arity) *slot++ = val_null(); /* 补默认参（slot2+na 起） */
    CallFrame *nf = ctx_frame_new(ctx);
    *nf = (CallFrame){ .closure = cl, .savedpc = f->code, .u = { 0 }, .base = base,
                       .want = 1, .defer_mark = ctx->defer_top };
    ctx_frame(ctx)->argc = (uint16_t)(nargs + 1); /* vararg：实参数（含 self 槽） */
    ctx->stack_top = base + (int)f->max_regs;
    const CoroResult r = vm_run(ctx, NULL, stop);
    ctx->stack_top = ctx->stack + saved_top;
    if (r == CORO_ERROR) {
        vm_rethrow_error(ctx); /* 外层 vm_run 内：longjmp 展开（stack.top 已恢复）；否则标志式返回 */
        *out = val_null();
        return true;
    }
    if (r == CORO_YIELDED) {
        runtime_error(ctx, fr, "cannot yield inside reflection invoke");
        *out = val_null();
        return true;
    }
    /* 嵌套 vm_run 内可能 ensure_stack realloc 栈 → base 悬垂，必须按偏移重算 */
    *out = ctx->stack[(ptrdiff_t)(base - ctx->stack)];
    return true;
}

/* 二元运算符重载分发：左操作数优先，其次右操作数。返回 true=已处理。 */
MICA_FORCEINLINE bool try_bin_overload(mica_context* ctx, CallFrame* fr, int name_idx, Value a, Value b,
                                       Value* out) {
    if (val_is_instance(a)) {
        if (call_overload(ctx, fr, a, name_idx, &b, 1, out)) return true;
    }
    if (val_is_instance(b)) {
        if (call_overload(ctx, fr, b, name_idx, &a, 1, out)) return true;
    }
    return false;
}

MICA_FORCEINLINE Value concat_values(mica_context* ctx, CallFrame* fr, Value a, Value b) {
    /* 两操作数皆字符串：直接拼接，省 value_to_cstr 的两份 malloc+strlen+复制。
       用 val_is_string(b)（MICA_TVISSTR 单比较）替代 `val_is_obj(b)&&type==OBJ_STRING`
       ——obj_string_val 恒用 MICA_TSTR 编码，语义等价且免解引用。 */
    if (MICA_LIKELY(val_is_string(a) && val_is_string(b))) {
        const ObjString* sa = (const ObjString *)val_as_obj(a);
        const ObjString* sb = (const ObjString *)val_as_obj(b);
        if ((size_t)sa->len > SIZE_MAX - (size_t)sb->len) {
            runtime_error(ctx, fr, "string too long");
            return val_null();
        }
        const size_t total = (size_t)sa->len + (size_t)sb->len;
        /* 直接构造拼接串并哈希**新内容**（不能把 sa->chars 当 total 字节传给 obj_string_new——
           它只有 sa->len 字节，fnv_hash/memcpy 会越界读）。 */
        ObjString* s = gc_alloc(ctx->gc, sizeof(ObjString) + total + 1, OBJ_STRING);
        s->len = (uint32_t)total;
        memcpy(s->chars, sa->chars, sa->len);
        memcpy(s->chars + sa->len, sb->chars, sb->len);
        s->chars[total] = '\0';
        s->hash = 0; /* 懒哈希：不预计算（OBJ_HASHED 位由 gc_alloc 清零），首次访问再算 */
        return obj_string_val(s);
    }
    char* sa = value_to_cstr(a);
    char* sb = value_to_cstr(b);
    size_t la = strlen(sa), lb = strlen(sb);
    if (la > SIZE_MAX - lb - 1) {
        free(sa);
        free(sb);
        runtime_error(ctx, fr, "string too long");
        return val_null();
    }
    char* buf = malloc(la + lb + 1);
    assert(buf != NULL);
    memcpy(buf, sa, la);
    memcpy(buf + la, sb, lb);
    buf[la + lb] = '\0';
    ObjString* s = obj_string_new(ctx->gc, buf, la + lb);
    free(buf);
    free(sa);
    free(sb);
    return obj_string_val(s);
}

/* OP_CONCAT：一次拼接 n 个值（各段字符串化，与 `+` 的字符串拼接语义一致）。
   两遍法：先算总长（字符串直接用 chars/len，其余 value_to_cstr），再一次性分配
   最终串拷贝——省去逐段 `+` 的 n-1 次中间 ObjString 分配。 */
MICA_FORCEINLINE Value concat_n(mica_context* ctx, CallFrame* fr, Instr* pc, Value* parts, int n) {
    fr->savedpc = pc;
    size_t total = 0;
    for (int i = 0; i < n; i++) {
        size_t l;
        if (val_is_obj(parts[i]) && val_as_obj(parts[i])->type == OBJ_STRING) {
            l = ((const ObjString *)val_as_obj(parts[i]))->len;
        } else if (val_is_num(parts[i])) {
            char tmp[48];
            l = value_num_to_buf(val_as_float(parts[i]), tmp, sizeof tmp); /* 0 = 回退 malloc */
            if (!l) {
                char* s = value_to_cstr(parts[i]);
                l = strlen(s);
                free(s);
            }
        } else {
            char* s = value_to_cstr(parts[i]);
            l = strlen(s);
            free(s);
        }
        if (l > SIZE_MAX - total) {
            runtime_error(ctx, fr, "string too long");
            return val_null();
        }
        total += l;
    }
    char* buf = malloc(total + 1);
    if (!buf) {
        runtime_error(ctx, fr, "out of memory in string concat");
        return val_null();
    }
    size_t off = 0;
    for (int i = 0; i < n; i++) {
        if (val_is_obj(parts[i]) && val_as_obj(parts[i])->type == OBJ_STRING) {
            const ObjString* s = (const ObjString *)val_as_obj(parts[i]);
            memcpy(buf + off, s->chars, s->len);
            off += s->len;
        } else if (val_is_num(parts[i])) {
            char tmp[48];
            const size_t fl = value_num_to_buf(val_as_float(parts[i]), tmp, sizeof tmp);
            if (fl) {
                memcpy(buf + off, tmp, fl);
                off += fl;
            } else {
                char* s = value_to_cstr(parts[i]);
                const size_t l = strlen(s);
                memcpy(buf + off, s, l);
                off += l;
                free(s);
            }
        } else {
            char* s = value_to_cstr(parts[i]);
            const size_t l = strlen(s);
            memcpy(buf + off, s, l);
            off += l;
            free(s);
        }
    }
    buf[total] = '\0';
    ObjString* r = obj_string_new(ctx->gc, buf, total);
    free(buf);
    return obj_string_val(r);
}

/* 字符串三态比较（<0 / 0 / >0）：memcmp + 长度后缀。纯字符串，无 Value 类型分支。 */
MICA_FORCEINLINE int cmp_strings(const ObjString* sa, const ObjString* sb) {
    size_t minlen = sa->len < sb->len ? sa->len : sb->len;
    int c = memcmp(sa->chars, sb->chars, minlen);
    if (c != 0) return c;
    return (sa->len < sb->len) ? -1 : (sa->len > sb->len) ? 1 : 0;
}

MICA_FORCEINLINE int compare_values(mica_context* ctx, CallFrame* fr, Instr* pc, Value a, Value b) {
    fr->savedpc = pc;
    /* int53：53 位内整数是 number 区 double（val_is_num），直接 double 比较。
       bigint（超限装箱）后续作为标准库类，不再内建数值——此处按非数值报错。 */
    if (MICA_LIKELY(val_both_num(a, b))) {
        const double da = val_as_float(a), db = val_as_float(b);
        return (da < db) ? -1 : (da > db) ? 1 : 0;
    }
    if (val_is_string(a) && val_is_string(b)) {
        return cmp_strings((const ObjString *)val_as_obj(a), (const ObjString *)val_as_obj(b));
    }
    runtime_error(ctx, fr, "cannot compare %s with %s", value_type_name(a), value_type_name(b));
    return 0;
}

/* 布尔上下文强制（严格，2026-08-10）：仅 bool 合法，其它任何类型（num/str/null/对象）
   一律运行期报错。vm_truthy_type_error 是 noinline 冷函数（报错路径防 i-cache 膨胀）。 */
MICA_NOINLINE static void vm_truthy_type_error(mica_context* ctx, CallFrame* fr, Instr* pc, Value v) {
    fr->savedpc = pc;
    runtime_error(ctx, fr, "cannot use %s as a boolean", value_type_name(v));
}

/* 布尔上下文严格解释：v 须是 bool（否则报错）；返回 v 的真值。重载比较结果等作条件用。 */
MICA_FORCEINLINE bool vm_truthy_strict(mica_context* ctx, CallFrame* fr, Instr* pc, Value v) {
    if (MICA_LIKELY(MICA_TVISBOOL(v))) return val_as_bool(v);
    vm_truthy_type_error(ctx, fr, pc, v);
    return false;
}

/* 比较-跳过慢路径（noinline）：任一操作数是实例 → 运算符重载优先。
   op: 0=LT 1=LE 2=GT 3=GE 4=EQ 5=NE。抽离热 case 防 i-cache 膨胀。 */
MICA_NOINLINE static bool vm_cmp_true_slow(mica_context* ctx, CallFrame* fr, Instr* pc, Value a,
                                           Value b, int op) {
    fr->savedpc = pc;
    Value ov;
    switch (op) {
    case 0: /* ISEQ */
        if (try_bin_overload(ctx, fr, OP_NAME_EQ, a, b, &ov)) return CTX_HAS_ERROR(ctx) ? false : vm_truthy_strict(ctx, fr, pc, ov);
        return value_eq(a, b);
    case 1: /* ISNE */
        if (try_bin_overload(ctx, fr, OP_NAME_NE, a, b, &ov)) return CTX_HAS_ERROR(ctx) ? false : vm_truthy_strict(ctx, fr, pc, ov);
        return !value_eq(a, b);
    case 2: /* ISLT */
        if (try_bin_overload(ctx, fr, OP_NAME_LT, a, b, &ov)) return CTX_HAS_ERROR(ctx) ? false : vm_truthy_strict(ctx, fr, pc, ov);
        return CTX_HAS_ERROR(ctx) ? false : compare_values(ctx, fr, pc, a, b) < 0;
    default: /* ISLE */
        if (try_bin_overload(ctx, fr, OP_NAME_LE, a, b, &ov)) return CTX_HAS_ERROR(ctx) ? false : vm_truthy_strict(ctx, fr, pc, ov);
        return CTX_HAS_ERROR(ctx) ? false : compare_values(ctx, fr, pc, a, b) <= 0;
    }
}

/* 比较-跳过（阶段 C）：返回 a op b 的真值，与 OP_LT 等语义一致（实例重载优先 + 数值/字符串）。
   op: 0=ISEQ 1=ISNE 2=ISLT 3=ISLE。IS* 族用——真则跳过下一条（解释器 pc++）。
   热路径（两操作数皆非实例，无重载）直接 value_eq/compare_values，跳过 try_bin_overload 预检查。 */
MICA_FORCEINLINE bool vm_cmp_true(mica_context* ctx, CallFrame* fr, Instr* pc, Value a, Value b, int op) {
    if (MICA_LIKELY(!val_is_instance(a) && !val_is_instance(b))) {
        switch (op) {
        case 0: return value_eq(a, b);
        case 1: return !value_eq(a, b);
        case 2: return compare_values(ctx, fr, pc, a, b) < 0;
        default: return compare_values(ctx, fr, pc, a, b) <= 0;
        }
    }
    return vm_cmp_true_slow(ctx, fr, pc, a, b, op);
}

/* 比较结果慢路径（noinline）：OP_EQ..OP_LE 中任一操作数是实例 → 重载优先。
   op: 0=EQ 1=NE 2=LT 3=LE。返回比较结果的 Value（bool / 错误 null）。
   重载可能 ensure_stack realloc 栈，调用方（vm_run case）须 VM_SYNC_FRAME() 后写回。 */
MICA_NOINLINE static Value vm_cmp_op_slow(mica_context* ctx, CallFrame* fr, Instr* pc, Value a,
                                          Value b, int op) {
    fr->savedpc = pc;
    Value ov;
    const int name_idx = op == 0 ? OP_NAME_EQ : op == 1 ? OP_NAME_NE : op == 2 ? OP_NAME_LT : OP_NAME_LE;
    if (try_bin_overload(ctx, fr, name_idx, a, b, &ov)) return CTX_HAS_ERROR(ctx) ? val_null() : ov;
    switch (op) {
    case 0: return val_bool(value_eq(a, b));
    case 1: return val_bool(!value_eq(a, b));
    default: {
        const int cc = compare_values(ctx, fr, pc, a, b);
        if (CTX_HAS_ERROR(ctx)) return val_null();
        return val_bool(op == 2 ? cc < 0 : cc <= 0);
    }
    }
}

/* 取对象长度（OP_LENGTH / ISLT_LEN 共用）：字符串 len / 数组 count；否则报错返回 false。 */
static bool vm_obj_length(mica_context* ctx, CallFrame* fr, Instr* pc, Value v, int64_t* out) {
    fr->savedpc = pc;
    if (val_is_obj(v)) {
        Obj* o = val_as_obj(v);
        if (o->type == OBJ_STRING) {
            *out = (int64_t)((const ObjString *)o)->len;
            return true;
        }
        if (o->type == OBJ_ARRAY) {
            *out = (int64_t)obj_array_count((const ObjArray *)o);
            return true;
        }
    }
    runtime_error(ctx, fr, "cannot take length of %s", value_type_name(v));
    return false;
}

/* 算术运算：每个运算符独立实现（不做"指令→运算符"的二次 switch 间接）。
   数值操作数优先（热路径）：两操作数皆数值直接算，跳过重载/字符串检查；
   非数值再试类实例重载（实例 __add__ 胜过另一操作数的字符串拼接）；最后报错。
   int 溢出提升 float；字符串 + 走拼接。 */
/* 算术慢路径（noinline）：非数值分支（运算符重载/字符串拼接/重复/报错）抽离热 case，
   防 i-cache 膨胀。慢路径可能压帧（重载）或 realloc 栈，调用方（vm_run case）须
   VM_SYNC_FRAME() 重新装载 fr/R。 */
MICA_NOINLINE static Value op_add_slow(mica_context* ctx, CallFrame* fr, Instr* pc, Value a, Value b) {
    fr->savedpc = pc; /* 错误行号/帧边界依赖 savedpc（热分派局部 pc 写回点） */
    if (MICA_LIKELY(val_both_num(a, b))) {
        return val_float(val_as_float(a) + val_as_float(b));
    }
    Value r;
    if (try_bin_overload(ctx, fr, OP_NAME_ADD, a, b, &r)) return r;
    if (val_is_string(a)) return concat_values(ctx, fr, a, b);
    if (val_is_string(b)) return concat_values(ctx, fr, a, b);
    runtime_error(ctx, fr, "cannot add %s with %s", value_type_name(a), value_type_name(b));
    return val_null();
}

MICA_FORCEINLINE Value op_add(mica_context* ctx, CallFrame* fr, Instr* pc, Value a, Value b) {
    if (MICA_LIKELY(val_both_num(a, b))) {
        return val_float(val_as_float(a) + val_as_float(b)); /* int53 内整数/float 直接 double */
    }
    return op_add_slow(ctx, fr, pc, a, b);
}

MICA_NOINLINE static Value op_sub_slow(mica_context* ctx, CallFrame* fr, Instr* pc, Value a, Value b) {
    fr->savedpc = pc;
    if (MICA_LIKELY(val_both_num(a, b))) {
        return val_float(val_as_float(a) - val_as_float(b));
    }
    Value r;
    if (try_bin_overload(ctx, fr, OP_NAME_SUB, a, b, &r)) return r;
    runtime_error(ctx, fr, "cannot subtract %s from %s", value_type_name(b), value_type_name(a));
    return val_null();
}

MICA_FORCEINLINE Value op_sub(mica_context* ctx, CallFrame* fr, Instr* pc, Value a, Value b) {
    if (MICA_LIKELY(val_both_num(a, b))) {
        return val_float(val_as_float(a) - val_as_float(b));
    }
    return op_sub_slow(ctx, fr, pc, a, b);
}

/* 字符串重复：s * n（n≤0 → 空串；溢出 → 运行期错误）。 */
MICA_FORCEINLINE Value repeat_string(mica_context* ctx, CallFrame* fr, ObjString* s, int64_t n) {
    if (n <= 0) return obj_string_val(obj_string_new(ctx->gc, "", 0));
    if (s->len > 0 && (uint64_t)n > (uint64_t)SIZE_MAX / s->len) {
        runtime_error(ctx, fr, "string too long");
        return val_null();
    }
    const size_t total = (size_t)s->len * (size_t)n;
    char* buf = malloc(total + 1);
    if (!buf) {
        runtime_error(ctx, fr, "out of memory in string repeat");
        return val_null();
    }
    for (int64_t i = 0; i < n; i++) memcpy(buf + (size_t)i * s->len, s->chars, s->len);
    buf[total] = '\0';
    ObjString* r = obj_string_new(ctx->gc, buf, total);
    free(buf);
    return obj_string_val(r);
}

MICA_NOINLINE static Value op_mul_slow(mica_context* ctx, CallFrame* fr, Instr* pc, Value a, Value b) {
    fr->savedpc = pc;
    if (MICA_LIKELY(val_both_num(a, b))) {
        return val_float(val_as_float(a) * val_as_float(b));
    }
    Value r;
    if (try_bin_overload(ctx, fr, OP_NAME_MUL, a, b, &r)) return r;
    /* 字符串乘法：s * n 与 n * s 均把字符串重复 n 次（如 "ab" * 3 == "ababab"） */
    if (val_is_string(a) && val_is_num(b)) {
        return repeat_string(ctx, fr, (ObjString *)val_as_obj(a), (int64_t)val_as_float(b));
    }
    if (val_is_num(a) && val_is_string(b)) {
        return repeat_string(ctx, fr, (ObjString *)val_as_obj(b), (int64_t)val_as_float(a));
    }
    runtime_error(ctx, fr, "cannot multiply %s with %s", value_type_name(a), value_type_name(b));
    return val_null();
}

MICA_FORCEINLINE Value op_mul(mica_context* ctx, CallFrame* fr, Instr* pc, Value a, Value b) {
    if (MICA_LIKELY(val_both_num(a, b))) {
        return val_float(val_as_float(a) * val_as_float(b));
    }
    return op_mul_slow(ctx, fr, pc, a, b);
}

MICA_NOINLINE static Value op_div_slow(mica_context* ctx, CallFrame* fr, Instr* pc, Value a, Value b) {
    fr->savedpc = pc;
    const bool an = val_is_num(a);
    const bool bn = val_is_num(b);
    if (an && bn) {
        const double da = val_as_float(a);
        const double db = val_as_float(b);
        if (db == 0) {
            runtime_error(ctx, fr, "division by zero");
            return val_null();
        }
        return val_float(da / db);
    }
    Value r;
    if (try_bin_overload(ctx, fr, OP_NAME_DIV, a, b, &r)) return r;
    runtime_error(ctx, fr, "cannot divide %s by %s", value_type_name(a), value_type_name(b));
    return val_null();
}

MICA_FORCEINLINE Value op_div(mica_context* ctx, CallFrame* fr, Instr* pc, Value a, Value b) {
    if (MICA_LIKELY(val_both_num(a, b))) {
        const double db = val_as_float(b);
        if (MICA_LIKELY(db != 0)) return val_float(val_as_float(a) / db);
    }
    return op_div_slow(ctx, fr, pc, a, b);
}

MICA_NOINLINE static Value op_idiv_slow(mica_context* ctx, CallFrame* fr, Instr* pc, Value a, Value b) {
    fr->savedpc = pc;
    if (val_both_num(a, b)) {
        const double db = val_as_float(b);
        if (db == 0) {
            runtime_error(ctx, fr, "division by zero");
            return val_null();
        }
        return val_float(floor(val_as_float(a) / db)); /* floor 除法（向下取整） */
    }
    Value r;
    if (try_bin_overload(ctx, fr, OP_NAME_IDIV, a, b, &r)) return r;
    runtime_error(ctx, fr, "'//' requires integer operands");
    return val_null();
}

MICA_FORCEINLINE Value op_idiv(mica_context* ctx, CallFrame* fr, Instr* pc, Value a, Value b) {
    if (MICA_LIKELY(val_both_num(a, b))) {
        const double db = val_as_float(b);
        if (MICA_LIKELY(db != 0)) return val_float(floor(val_as_float(a) / db));
    }
    return op_idiv_slow(ctx, fr, pc, a, b);
}

MICA_NOINLINE static Value op_mod_slow(mica_context* ctx, CallFrame* fr, Instr* pc, Value a, Value b) {
    fr->savedpc = pc;
    if (val_both_num(a, b)) {
        const double db = val_as_float(b);
        if (db == 0) {
            runtime_error(ctx, fr, "division by zero in '%%'");
            return val_null();
        }
        double r2 = fmod(val_as_float(a), db);
        if (r2 != 0 && ((r2 < 0) != (db < 0))) r2 += db; /* 数学模：结果符号同除数 */
        return val_float(r2);
    }
    Value r;
    if (try_bin_overload(ctx, fr, OP_NAME_MOD, a, b, &r)) return r;
    runtime_error(ctx, fr, "'%%' requires integer operands");
    return val_null();
}

MICA_FORCEINLINE Value op_mod(mica_context* ctx, CallFrame* fr, Instr* pc, Value a, Value b) {
    if (MICA_LIKELY(val_both_num(a, b))) {
        const double db = val_as_float(b);
        if (MICA_LIKELY(db != 0)) {
            double r2 = fmod(val_as_float(a), db);
            if (r2 != 0 && ((r2 < 0) != (db < 0))) r2 += db;
            return val_float(r2);
        }
    }
    return op_mod_slow(ctx, fr, pc, a, b);
}

MICA_NOINLINE static Value op_pow_slow(mica_context* ctx, CallFrame* fr, Instr* pc, Value a, Value b) {
    fr->savedpc = pc;
    if (val_both_num(a, b)) {
        return val_float(pow(val_as_float(a), val_as_float(b)));
    }
    Value r;
    if (try_bin_overload(ctx, fr, OP_NAME_POW, a, b, &r)) return r;
    runtime_error(ctx, fr, "cannot exponentiate %s by %s", value_type_name(a), value_type_name(b));
    return val_null();
}

MICA_FORCEINLINE Value op_pow(mica_context* ctx, CallFrame* fr, Instr* pc, Value a, Value b) {
    if (MICA_LIKELY(val_both_num(a, b))) {
        /* int 已删：number 一律 double。整值结果 (double)(int64_t)d == d（int64 内无损往返，
           且 -0.0 保留符号），旧装箱分支是死代码。 */
        return val_float(pow(val_as_float(a), val_as_float(b)));
    }
    return op_pow_slow(ctx, fr, pc, a, b);
}

MICA_NOINLINE static Value value_neg_slow(mica_context* ctx, CallFrame* fr, Instr* pc, Value v) {
    fr->savedpc = pc;
    if (val_is_num(v)) {
        return val_float(-val_as_float(v));
    }
    if (val_is_instance(v)) {
        Value r;
        if (call_overload(ctx, fr, v, OP_NAME_NEG, NULL, 0, &r)) return r;
    }
    runtime_error(ctx, fr, "cannot negate %s", value_type_name(v));
    return val_null();
}

MICA_FORCEINLINE Value value_neg(mica_context* ctx, CallFrame* fr, Instr* pc, Value v) {
    if (MICA_LIKELY(val_is_float(v))) return val_float(-val_as_float(v)); /* number 区（int53/float）直接取负 */
    return value_neg_slow(ctx, fr, pc, v);
}

/* 原始键快捷宏（(hash, chars, len) 三参；标识符池（String）直接查找，免包 Value）。 */
#define NAME_KV(name) (name)->hash, str_data(name), str_len(name)

/* 调用原生属性 getter/setter（mica_register_property 注入）：getter 收 self（args[0]）、
   setter 收 self+value（args[0..1]）。结果写 *out；出错（CTX_HAS_ERROR(ctx)）返回 false。 */
static bool invoke_native_prop(mica_context *ctx, ObjNativeClosure *nat, Value *args, uint32_t argc,
                               Value *out) {
    mica_value nres[MICA_NATIVE_MAX_RESULTS];
    const uint32_t n = nat->func(ctx, (mica_value *)(void *)args, argc, nres);
    if (CTX_HAS_ERROR(ctx)) return false;
    *out = (n > 0) ? val_unwrap(nres[0]) : val_null();
    return true;
}

/* `is` 类检查：v 是否 d（OBJ_CLASS 或 OBJ_NATIVE_CLASS）的实例（沿继承链）。
   普通类与原生类是独立类型（不同结构、不同继承链），分别处理。 */
static bool vm_is_instance_of(Value v, Value d) {
    if (!val_is_obj(d)) return false;
    Obj *do_ = val_as_obj(d);
    if (do_->type == OBJ_CLASS) {
        if (!val_is_instance(v)) return false;
        ObjClass *k = ((ObjInstance *)val_as_obj(v))->klass;
        ObjClass *want = (ObjClass *)do_;
        while (k) {
            if (k == want) return true;
            k = k->superclass;
        }
        return false;
    }
    if (do_->type == OBJ_NATIVE_CLASS) {
        /* 原生类无继承链：实例 klass 与目标类直接比较 */
        return val_is_obj(v) && val_as_obj(v)->type == OBJ_NATIVE_INSTANCE &&
               ((ObjNativeInstance *)val_as_obj(v))->klass == (ObjNativeClass *)do_;
    }
    return false;
}

/* is 的内置类型名匹配（OP_IS_TYPE 的 ObjString 分支；类/枚举 case 走其他路径）。
   按长度分组 + 哈希比较（strlit.h），拒绝绝大多数不匹配，命中再 memcmp 兜底。 */
MICA_FORCEINLINE bool value_matches_type_name(Value v, ObjString* name) {
    switch (name->len) {
    case 3:
        if (NAME_EQ(name, H_INT, "int")) return val_is_num(v);
        if (NAME_EQ(name, H_MAP, "map")) return val_is_map(v);
        break;
    case 4:
        if (NAME_EQ(name, H_BOOL, "bool")) return val_is_bool(v);
        if (NAME_EQ(name, H_NULL, "null")) return val_is_null(v);
        if (NAME_EQ(name, H_ENUM, "enum"))
            return val_is_obj(v) && val_as_obj(v)->type == OBJ_ENUM_VAL;
        break;
    case 5:
        if (NAME_EQ(name, H_FLOAT, "float")) return val_is_float(v);
        if (NAME_EQ(name, H_ARRAY, "array")) return val_is_array(v);
        if (NAME_EQ(name, H_ERROR, "error"))
            return val_is_obj(v) && val_as_obj(v)->type == OBJ_ERROR;
        break;
    case 6:
        if (NAME_EQ(name, H_NUMBER, "number")) return val_is_num(v);
        if (NAME_EQ(name, H_STRING, "string")) return val_is_string(v);
        if (NAME_EQ(name, H_OBJECT, "object")) return val_is_obj(v);
        break;
    case 8:
        if (NAME_EQ(name, H_FUNCTION, "function")) {
            if (!val_is_obj(v)) return false;
            const ObjType t = val_as_obj(v)->type;
            return t == OBJ_CLOSURE || t == OBJ_NATIVE_CLOSURE || t == OBJ_BUILTIN_METHOD;
        }
        break;
    case 9:
        if (NAME_EQ(name, H_COROUTINE, "coroutine"))
            return val_is_obj(v) && val_as_obj(v)->type == OBJ_CONTEXT &&
                   CTX_IS_CORO((mica_context *)val_as_obj(v));
        break;
    default:
        break;
    }
    return false;
}

/* ---------- get_member: per-object-type dispatch ----------
   The old giant switch is a function-pointer table (k_get_member[type]);
   names come from proto's identifier pool (SSO String), so lookups use the
   raw (hash, chars, len) *_kv variants - no Value/ObjString wrapper. */

/* 从标识符池 String 取 (hash, chars, len) 三参快捷宏（定义见慢路径区前部）。
   chars 用 str_data（池内稳定指针，非栈临时——跨嵌套调用安全）。 */

typedef Value (*GetMemberFn)(mica_context* ctx, CallFrame* fr, Value obj, const String* name);

/* ---------- 内建方法单例缓存（VM 热路径优化） ----------
   get_member 每次访问 `a.push` / `m.has` / `s.trim` 都 obj_builtin_method_new（gc_alloc）
   ——热循环里这是**每次**的 GC 分配 + GC 压力来源（bench_vm 的 member/builtin_method
   两项主要开销）。ObjBuiltinMethod 只含 mica_native_fn fn（无状态），可安全共享 permanent
   单例：按 fn 缓存（fn 指针唯一确定一个内建方法），首次 miss 创建 permanent 对象，
   之后命中直接返回缓存值 → 消除该分配的分配开销与 GC 压力。

   缓存数组挂 **gc->bm_cache（每 Gc 一份）**：permanent 对象属某个 Gc，若用全局缓存在
   Gc 释放后会悬空（test_gc 频繁新建 Gc，且 malloc 会复用同地址 → UAF），故必须 per-Gc。
   数组本体由 gc_deinit 释放；其中的 permanent 对象由 permanent 链统一释放。
   内建方法总数有限（array/string/map/weakref ≈ 25 个），线性查表足够（命中率 100%）。 */
#define BM_CACHE_MAX 32
typedef struct {
    mica_native_fn fn;
    ObjBuiltinMethod *bm;
} BmCacheEntry;
typedef struct {
    int n;
    BmCacheEntry e[BM_CACHE_MAX];
} BmCache;

static Value builtin_method_value(Gc *gc, mica_native_fn fn) {
    BmCache *c = (BmCache *)gc->bm_cache;
    if (MICA_UNLIKELY(c == NULL)) {
        c = (BmCache *)calloc(1, sizeof(BmCache));
        if (MICA_UNLIKELY(c == NULL))
            return obj_builtin_method_val(obj_builtin_method_new(gc, fn)); /* OOM 兜底 */
        gc->bm_cache = c;
    }
    for (int i = 0; i < c->n; i++) {
        if (c->e[i].fn == fn) return obj_builtin_method_val(c->e[i].bm); /* 命中：免分配 */
    }
    if (MICA_LIKELY(c->n < BM_CACHE_MAX)) {
        ObjBuiltinMethod *bm = obj_builtin_method_new_permanent(gc, fn);
        c->e[c->n].fn = fn;
        c->e[c->n].bm = bm;
        c->n++;
        return obj_builtin_method_val(bm);
    }
    /* 缓存满（内建方法 > 32，理论上不会）→ 退回每次分配（正确性优先） */
    return obj_builtin_method_val(obj_builtin_method_new(gc, fn));
}

static Value gm_map(mica_context* ctx, CallFrame* fr, Value obj, const String* name) {
    (void)fr;
    Value out;
    if (obj_map_get_by_string((const ObjMap *)val_as_obj(obj), str_data(name), str_len(name),
                              name->hash, &out))
        return out;
    /* key absent -> builtin method (user key takes precedence over method name) */
    mica_native_fn fn = find_builtin_method_kv(OBJ_MAP, NAME_KV(name));
    if (fn) return builtin_method_value(ctx->gc, fn); /* 单例缓存：免每次 gc_alloc */
    return val_null();
}

static Value gm_array(mica_context* ctx, CallFrame* fr, Value obj, const String* name) {
    if (STR_EQ(name, H_LENGTH, "length"))
        return val_int((int64_t)obj_array_count((const ObjArray *)val_as_obj(obj)));
    mica_native_fn fn = find_builtin_method_kv(OBJ_ARRAY, NAME_KV(name));
    if (fn) return builtin_method_value(ctx->gc, fn); /* 单例缓存：免每次 gc_alloc */
    runtime_error(ctx, fr, "array has no member '%s'", str_data(name));
    return val_null();
}

static Value gm_string(mica_context* ctx, CallFrame* fr, Value obj, const String* name) {
    if (STR_EQ(name, H_LENGTH, "length"))
        return val_int((int64_t)((const ObjString *)val_as_obj(obj))->len);
    mica_native_fn fn = find_builtin_method_kv(OBJ_STRING, NAME_KV(name));
    if (fn) return builtin_method_value(ctx->gc, fn); /* 单例缓存：免每次 gc_alloc */
    runtime_error(ctx, fr, "string has no member '%s'", str_data(name));
    return val_null();
}

static Value gm_error(mica_context* ctx, CallFrame* fr, Value obj, const String* name) {
    if (STR_EQ(name, H_MESSAGE, "message"))
        return obj_string_val(((ObjError *)val_as_obj(obj))->message);
    runtime_error(ctx, fr, "error has no member '%s'", str_data(name));
    return val_null();
}

static Value gm_enum_val(mica_context* ctx, CallFrame* fr, Value obj, const String* name) {
    if (STR_EQ(name, H_PAYLOAD, "payload")) {
        ObjEnumVal* e = (ObjEnumVal *)val_as_obj(obj);
        ObjArray* arr = obj_array_new(ctx->gc, e->n_payload);
        for (size_t i = 0; i < e->n_payload; i++) obj_array_push(ctx->gc, arr, e->payload[i]);
        return obj_array_val(arr);
    }
    runtime_error(ctx, fr, "enum value has no member '%s'", str_data(name));
    return val_null();
}

static Value gm_instance(mica_context* ctx, CallFrame* fr, Value obj, const String* name) {
    ObjInstance* inst = (ObjInstance *)val_as_obj(obj);
    const int idx = obj_class_field_index(inst->klass, name);
    if (idx >= 0) return inst->fields[idx]; /* 声明字段（未设 = null） */
    ObjClosure* m = obj_class_find_method_kv(inst->klass, NAME_KV(name));
    if (m) return obj_closure_val(m); /* 普通方法闭包（不绑定；调用时 receiver 作 self 首参） */
    const char* cn = inst->klass->name ? inst->klass->name->chars : "?";
    runtime_error(ctx, fr, "class %s has no member '%s'", cn, str_data(name));
    return val_null();
}

static Value gm_native_instance(mica_context* ctx, CallFrame* fr, Value obj, const String* name) {
    ObjNativeInstance* inst = (ObjNativeInstance *)val_as_obj(obj);
    ObjClosure* m = obj_native_class_find_method_kv(inst->klass, NAME_KV(name));
    if (m) return obj_closure_val(m); /* 原生实例方法（receiver 作 self 首参） */
    runtime_error(ctx, fr, "native instance has no member '%s'", str_data(name));
    return val_null();
}

static Value gm_native_class(mica_context* ctx, CallFrame* fr, Value obj, const String* name) {
    ObjNativeClass* klass = (ObjNativeClass *)val_as_obj(obj);
    Value m = obj_native_class_find_static_kv(klass, NAME_KV(name));
    if (!val_is_null(m)) return m; /* ObjClosure / ObjNativeClosure */
    runtime_error(ctx, fr, "native class has no static member '%s'", str_data(name));
    return val_null();
}

static Value gm_class(mica_context* ctx, CallFrame* fr, Value obj, const String* name) {
    ObjClass* klass = (ObjClass *)val_as_obj(obj);
    Value m = obj_class_find_static_kv(klass, NAME_KV(name));
    if (!val_is_null(m)) return m; /* ObjClosure or ObjNativeClosure (Assembly.load etc) */
    runtime_error(ctx, fr, "class has no static member '%s'", str_data(name));
    return val_null();
}

static Value gm_coroutine(mica_context* ctx, CallFrame* fr, Value obj, const String* name) {
    (void)obj;
    /* 协程实例（OBJ_CONTEXT）不再支持成员方法；语言 API = coroutine.resume(co) 等 */
    runtime_error(ctx, fr, "coroutine has no member '%s'; use coroutine.resume/cancel/status(co)",
                  str_data(name));
    return val_null();
}

static Value gm_weakref(mica_context* ctx, CallFrame* fr, Value obj, const String* name) {
    (void)obj;
    mica_native_fn fn = find_builtin_method_kv(OBJ_WEAKREF, NAME_KV(name));
    if (fn) return builtin_method_value(ctx->gc, fn); /* 单例缓存：免每次 gc_alloc */
    runtime_error(ctx, fr, "weakref has no member '%s'", str_data(name));
    return val_null();
}

/* 反射对象成员访问（LREFL_* 自定义类型）已移入 librefl.c（经 vm_register_custom_member 注册）。 */

/* unhandled types (non-container / non-builtin-method object): member access error */
static Value gm_default(mica_context* ctx, CallFrame* fr, Value obj, const String* name) {
    runtime_error(ctx, fr, "cannot access member '%s' on %s", str_data(name), value_type_name(obj));
    return val_null();
}

/* member-access dispatch table indexed by ObjType（核心类型；自定义类型经 k_custom_member） */
static const GetMemberFn k_get_member[OBJ_TYPE_COUNT] = {
    [OBJ_MAP] = gm_map,
    [OBJ_ARRAY] = gm_array,
    [OBJ_STRING] = gm_string,
    [OBJ_ERROR] = gm_error,
    [OBJ_ENUM_VAL] = gm_enum_val,
    [OBJ_INSTANCE] = gm_instance,
    [OBJ_NATIVE_INSTANCE] = gm_native_instance,
    [OBJ_CLASS] = gm_class,
    [OBJ_NATIVE_CLASS] = gm_native_class,
    [OBJ_CONTEXT] = gm_coroutine,
    [OBJ_WEAKREF] = gm_weakref,
};

/* 自定义类型（>= OBJ_TYPE_COUNT，如 librefl 反射对象）成员访问处理器注册表。
   lib 模块在 librefl_init 时注册；indexed by type（GC_VTABLE_SLOTS 槽）。 */
static VmCustomMemberFn k_custom_member[GC_VTABLE_SLOTS];

void vm_register_custom_member(uint32_t type, VmCustomMemberFn fn) {
    if (type < GC_VTABLE_SLOTS) k_custom_member[type] = fn;
}

MICA_FORCEINLINE Value get_member(mica_context* ctx, CallFrame* fr, Instr* pc, Value obj,
                                  const String* name) {
    fr->savedpc = pc;
    if (val_is_obj(obj)) {
        Obj* o = val_as_obj(obj);
        if (o->type < OBJ_TYPE_COUNT && k_get_member[o->type]) {
            return k_get_member[o->type](ctx, fr, obj, name);
        }
        if (o->type >= OBJ_TYPE_COUNT && o->type < GC_VTABLE_SLOTS && k_custom_member[o->type]) {
            return k_custom_member[o->type](ctx, fr, obj, name);
        }
    }
    return gm_default(ctx, fr, obj, name);
}

MICA_FORCEINLINE void set_member(mica_context* ctx, CallFrame* fr, Instr* pc, Value obj,
                                 const String* name, Value v) {
    fr->savedpc = pc;
    if (val_is_map(obj)) {
        obj_map_set_by_string(ctx->gc, (ObjMap *)val_as_obj(obj), str_data(name), str_len(name),
                              name->hash, v);
        return;
    }
    if (val_is_instance(obj)) {
        if (!obj_instance_set_field(ctx->gc, (ObjInstance *)val_as_obj(obj), name, v))
            runtime_error(ctx, fr, "instance has no field '%s'", str_data(name));
        return;
    }
    runtime_error(ctx, fr, "cannot set member '%s' on %s", str_data(name), value_type_name(obj));
}

MICA_FORCEINLINE Value index_get(mica_context* ctx, CallFrame* fr, Instr* pc, Value obj, Value idx) {
    fr->savedpc = pc;
    if (val_is_instance(obj)) {
        Value r;
        if (call_overload(ctx, fr, obj, OP_NAME_INDEX, &idx, 1, &r)) return r;
    }
    if (val_is_array(obj)) {
        if (!val_is_num(idx)) {
            runtime_error(ctx, fr, "array index must be an int");
            return val_null();
        }
        ObjArray* a = (ObjArray *)val_as_obj(obj);
        int64_t i;
        if (!array_index_of(ctx, fr, NULL, a, idx, &i)) return val_null();
        return obj_array_get(a, (size_t)i);
    }
    if (val_is_map(obj)) {
        if (!val_is_string(idx)) {
            runtime_error(ctx, fr, "map index must be a string");
            return val_null();
        }
        ObjString *ks = (ObjString *)val_as_obj(idx);
        Value out;
        if (obj_map_get_kv((const ObjMap *)val_as_obj(obj), obj_string_hash(ks), ks->chars, ks->len, &out))
            return out;
        return val_null();
    }
    runtime_error(ctx, fr, "cannot index %s with %s", value_type_name(obj), value_type_name(idx));
    return val_null();
}

MICA_FORCEINLINE void index_set(mica_context* ctx, CallFrame* fr, Instr* pc, Value obj, Value idx,
                                Value v) {
    fr->savedpc = pc;
    if (val_is_instance(obj)) {
        Value args[2] = { idx, v };
        Value r;
        if (call_overload(ctx, fr, obj, OP_NAME_INDEX_SET, args, 2, &r)) return;
    }
    if (val_is_array(obj)) {
        if (!val_is_num(idx)) {
            runtime_error(ctx, fr, "array index must be an int");
            return;
        }
        ObjArray* a = (ObjArray *)val_as_obj(obj);
        int64_t i;
        if (!array_index_of(ctx, fr, NULL, a, idx, &i)) return;
        obj_array_set(ctx->gc, a, (size_t)i, v);
        return;
    }
    if (val_is_map(obj)) {
        if (!val_is_string(idx)) {
            runtime_error(ctx, fr, "map index must be a string");
            return;
        }
        ObjString *ks = (ObjString *)val_as_obj(idx);
        obj_map_set_kv(ctx->gc, (ObjMap *)val_as_obj(obj), obj_string_hash(ks), ks->chars, ks->len, v);
        return;
    }
    runtime_error(ctx, fr, "cannot assign index on %s with %s", value_type_name(obj),
                  value_type_name(idx));
}

/* 调用结果模式应用（native / 内置方法 / 类 native 构造共用）：
   0=丢弃 / 127=可变 / n=固定（不足补 null）。 */
static void set_call_result(mica_context* ctx, Value* base, int mode, uint32_t n) {
    if (mode == RESULT_DISCARD) {
        ctx->stack_top = base;
    } else if (mode == RESULT_VARARGS) {
        ctx->stack_top = base + (int)n;
    } else {
        for (uint32_t i = n; i < (uint32_t)mode; i++) base[i] = val_null();
        ctx->stack_top = base + (int)mode;
    }
}

/* 调用分派：OP_CALL / OP_FASTCALL 共用。
   unprotected：本调用未受 try/catch 保护（RESULT_PROTECT 未置）——返回错误值时按"未捕获错误"退出。
   热路径 OBJ_CLOSURE 用 LIKELY 快速分支；
   其余类型一次取值后走 switch（免重复 val_is_obj + type 判断）。
   receiver/self 已由 codegen 显式排入 slot1（argc 含 receiver）；内置/协程方法也从 slot1 读 receiver。
   所有分支处理或报错（CTX_HAS_ERROR(ctx)）。 */
static void vm_call_dispatch(mica_context* ctx, CallFrame* fr, Instr* pc, Value fn, int a, int argc,
                             int mode, bool unprotected) {
    fr->savedpc = pc; /* caller 恢复点：压 callee 帧 / native 直调前持久化（返回后从 savedpc 续） */
    Obj* o = val_is_obj(fn) ? val_as_obj(fn) : NULL;
    if (MICA_LIKELY(o && o->type == OBJ_CLOSURE)) {
        ObjClosure* cl = (ObjClosure *)o;
        Proto* f = cl->proto;
        /* ⚠️ 顺序：先 ensure_stack（按 max(max_regs, arity+1) 定界）再写默认参数槽。
           原实现先填 new_base[1+i] 到 arity-1 再 ensure_stack(base+max_regs)，
           arity > max_regs 时会越界写栈块之外。 */
        const ptrdiff_t base_off = (ptrdiff_t)((fr->base + a) - ctx->stack);
        const int need = (int)f->max_regs > (int)f->arity + 1 ? (int)f->max_regs
                                                             : (int)f->arity + 1;
        if (!ensure_stack(ctx, ctx->stack + base_off + need)) {
            runtime_error(ctx, fr, "stack overflow (call too deep)");
            return;
        }
        Value* new_base = ctx->stack + base_off; /* ensure_stack realloc 后重算（指针） */
        /* 追踪模型（T2 起）：调用不内联——解释器压帧续跑；
           已编译 trace 的 CALL 路径（T4 内联）另行接入。 */
        for (uint32_t i = (uint32_t)argc; i < f->arity; i++) new_base[1 + i] = val_null();
        if (ctx->frame_count >= MICA_FRAME_MAX_SOFT) {
            runtime_error(ctx, fr, "stack overflow (too many frames)");
            return;
        }
        CallFrame *cf = ctx_frame_new(ctx);
        *cf = (CallFrame){
            .closure = cl, .savedpc = f->code, .u = { 0 }, .base = new_base,
            .want = (uint8_t)mode, .flags = (uint8_t)(unprotected ? CF_UNPROTECTED : 0),
            .defer_mark = ctx->defer_top,
        };
        cf->argc = (uint16_t)argc; /* vararg 取多余实参用 */
        ctx->stack_top = new_base + (int)f->max_regs;
        return;
    }
    if (!o) {
        runtime_error(ctx, fr, "attempt to call a non-function value (%s)", value_type_name(fn));
        return;
    }
    switch (o->type) {
    case OBJ_NATIVE_CLOSURE: {
        ObjNativeClosure* nat = (ObjNativeClosure *)o;
        if (nat->internal) {
            const char* mod = (fr->closure && fr->closure->proto->module)
                                  ? fr->closure->proto->module->chars
                                  : "";
            if (strncmp(mod, "System.", 7) != 0) {
                runtime_error(ctx, fr,
                              "System.Builtin is internal: '%s' can only be called inside the "
                              "standard library",
                              nat->name->chars);
                return;
            }
        }
        /* 结果 staging（旧 user_native_bridge 语义）：宿主 native 的 results 缓冲独立于实参
           寄存器（多返回值会与 args 区重叠覆盖），上限 MICA_NATIVE_MAX_RESULTS=8。 */
        mica_value nres[MICA_NATIVE_MAX_RESULTS];
        /* 统一约定（2026-08-15）：slot1=self。方法 native（is_method）receiver 作首参
           （args=base+a+1、nargs=argc 含 receiver）；普通 native slot1=null self，
           args=base+a+2、nargs=argc-1。 */
        const bool is_meth = nat->is_method;
        mica_value* args = (mica_value *)(void *)(fr->base + a + (is_meth ? 1 : 2));
        const uint32_t nargs = (uint32_t)argc - (is_meth ? 0u : 1u);
        const uint32_t n = nat->func(ctx, args, nargs, nres);
        /* native 内部可能嵌套调用 VM（Assembly.load 加载 .mlib 跑模块 init / 反射 invoke）
           触发 ensure_stack realloc 值栈 → 旧的 fr->base 悬垂。须重读 fr->base（ensure_stack
           已按偏移更新帧的 base 字段）再写结果，否则写已释放内存 → 堆损坏（Heisenbug）。 */
        Value* base = fr->base;
        const uint32_t cn = n > MICA_NATIVE_MAX_RESULTS ? MICA_NATIVE_MAX_RESULTS : n;
        for (uint32_t i = 0; i < cn; i++) base[a + i] = val_unwrap(nres[i]);
        set_call_result(ctx, base + a, mode, cn);
        return;
    }
    case OBJ_BUILTIN_METHOD: {
        /* 内置类型成员方法（array/string/map）：receiver 已在 slot1（codegen 注入）。
           native 约定 args[0]=receiver、argc=用户实参数+1（含 receiver）。 */
        ObjBuiltinMethod* bm = (ObjBuiltinMethod *)o;
        Value* base = fr->base + a;
        if (argc < 1) {
            runtime_error(ctx, fr, "builtin method requires a receiver");
            return;
        }
        const uint32_t n = bm->fn(ctx, (mica_value *)&base[1], (uint32_t)argc,
                                  (mica_value *)base);
        base = fr->base + a; /* bm->fn 未来可能嵌套调用 VM 触发 ensure_stack realloc → 重读 base */
        set_call_result(ctx, base, mode, n);
        return;
    }
    case OBJ_CLASS: {
        /* 类调用 = 构造：建实例，跑字段默认值（若有），再跑 init，结果为实例 */
        ObjClass* klass = (ObjClass *)o;
        if (klass->native_ctor) {
            Value* base = fr->base + a;
            /* 统一约定：类调用 slot1=null self、实参从 slot2。native_ctor 收纯实参。 */
            const uint32_t n = klass->native_ctor(ctx, (mica_value *)&base[2],
                                                  (uint32_t)argc - 1, (mica_value *)base);
            set_call_result(ctx, base, mode, n);
            return;
        }
        ObjInstance* inst = obj_instance_new(ctx->gc, klass);
        ObjClosure* defaults = klass->defaults;
        ObjString* init_name = ctx->rt->init_name; /* 缓存到 runtime（跨 runtime 不能缓存函数级 static） */
        if (!init_name) {
            init_name = obj_string_new_permanent(ctx->gc, "init", 4);
            ctx->rt->init_name = init_name;
        }
        ObjClosure* init = obj_class_find_method(klass, init_name);
        fr->base[a] = obj_instance_val(inst);
        if (defaults || init) {
            ObjClosure* entry = defaults ? defaults : init;
            /* ⚠️ 顺序：ensure_stack（按 max(max_regs, arity+1) 定界）必须在写默认参数槽之前，
               否则 arity > max_regs 时越界写栈块之外。 */
            const ptrdiff_t base_off = (ptrdiff_t)((fr->base + a) - ctx->stack);
            const int need = (int)entry->proto->max_regs > (int)entry->proto->arity + 1
                                 ? (int)entry->proto->max_regs
                                 : (int)entry->proto->arity + 1;
            if (!ensure_stack(ctx, ctx->stack + base_off + need)) {
                runtime_error(ctx, fr, "stack overflow (constructor)");
                return;
            }
            Value* new_base = ctx->stack + base_off; /* ensure_stack realloc 后重算（指针） */
            /* 统一约定（2026-08-15）：类调用实参已在 slot2+（slot1=null self 由 codegen 排放），
               无需 memmove。slot1 覆写为实例（self）、slot0 覆写为入口闭包（defaults/init）。 */
            new_base[1] = obj_instance_val(inst);
            *new_base = obj_closure_val(entry);
            for (int i = argc; i < (int)entry->proto->arity; i++) new_base[1 + i] = val_null();
            if (ctx->frame_count >= MICA_FRAME_MAX_SOFT) {
                runtime_error(ctx, fr, "stack overflow (too many frames)");
                return;
            }
            CallFrame nf = (CallFrame){
                .closure = entry, .savedpc = entry->proto->code, .u = { 0 }, .base = new_base,
                .want = RESULT_DISCARD, .flags = 0, .defer_mark = ctx->defer_top,
            };
            nf.u.ctor.after = (defaults ? init : NULL);
            nf.u.ctor.constructed = inst;
            nf.cargc = (uint16_t)(argc - 1); /* 用户实参数（不含 self 槽） */
            nf.cwant = (uint8_t)mode;
            *ctx_frame_new(ctx) = nf;
            ctx->stack_top = new_base + (int)entry->proto->max_regs;
        }
        return;
    }
    case OBJ_NATIVE_CLASS: {
        /* 原生类调用 = 构造：建零初始化 userdata 的原生实例（v1：无 init/默认值/custom ctor）。 */
        ObjNativeClass* nk = (ObjNativeClass *)o;
        ObjNativeInstance* inst = obj_native_instance_new(ctx->gc, nk);
        fr->base[a] = obj_native_instance_val(inst);
        set_call_result(ctx, fr->base + a, mode, 1);
        return;
    }
    case OBJ_CONTEXT: {
        /* 协程实例（OBJ_CONTEXT）不可直接调用；语言 API = coroutine.resume(co) 等 */
        (void)o;
        runtime_error(ctx, fr, "coroutine is not directly callable; use coroutine.resume(co)");
        return;
    }
    default:
        runtime_error(ctx, fr, "attempt to call a non-function value (%s)", value_type_name(fn));
        return;
    }
}

/* ---------- 冷 op 处理器（Phase 1：从 vm_run 主循环外移，缩小热循环 i-cache） ----------
   错误/枚举/协程/defer/闭包构造等罕见 op 各自成独立函数；主循环 case 只留一行调用。
   均不 realloc 栈（vm_cold_defer_run 嵌套 vm_run 例外），R 在函数内派生自 fr->base。
   注意：这些函数在热循环外，无需 MICA_FORCEINLINE；错误路径按冷分支处理。 */

static void vm_cold_error(mica_context* ctx, CallFrame* fr, Instr in) {
    const ObjString* msg = (const ObjString *)val_as_obj(fr->closure->proto->constants[instr_a(in)]);
    runtime_error(ctx, fr, "%s", msg->chars);
}

static void vm_cold_new_error(mica_context* ctx, CallFrame* fr, Instr in) {
    Value* R = fr->base;
    char* s = value_to_cstr(R[instr_b(in)]);
    /* unsafe 窗口：msg 只被 C 局部引用，obj_error_new 内 gc_alloc 可能触发收集 → 悬垂 */
    gc_begin_unsafe(ctx->gc);
    ObjString* msg = obj_string_new(ctx->gc, s, strlen(s));
    free(s);
    ObjError* e = obj_error_new(ctx->gc, msg, NULL, NULL, NULL, NULL, 0);
    R[instr_a(in)] = obj_error_val(e);
    gc_end_unsafe(ctx->gc);
}

static void vm_cold_new_error_named(mica_context* ctx, CallFrame* fr, Instr in) {
    const Proto* ch = fr->closure->proto;
    Value* R = fr->base;
    const int n = instr_b(in);
    const int first = instr_c(in);
    const Instr info = *fr->savedpc++;
    if (instr_op(info) != OP_ERROR_INFO) {
        runtime_error(ctx, fr, "malformed error descriptor");
        return;
    }
    ObjString* set_name = const_str_or_null(ch, instr_a(info));
    ObjString* case_name = const_str_or_null(ch, instr_b(info));
    ObjString** pnames = NULL;
    Value* payload = NULL;
    if (n > 0) {
        payload = malloc((size_t)n * sizeof(Value));
        pnames = malloc((size_t)n * sizeof(ObjString *));
        assert(payload != NULL && pnames != NULL);
        for (int i = 0; i < n; i++) {
            payload[i] = R[first + i];
            const Instr d = *fr->savedpc++;
            if (instr_op(d) != OP_ERROR_INFO) {
                runtime_error(ctx, fr, "malformed error descriptor");
                free(payload);
                free(pnames);
                payload = NULL;
                pnames = NULL;
                break;
            }
            pnames[i] = const_str_or_null(ch, instr_a(d));
        }
    }
    if (CTX_HAS_ERROR(ctx)) return;
    ObjString* message = NULL;
    int msg_idx = -1;
    for (int i = 0; i < n; i++) {
        if (pnames[i] && strcmp(pnames[i]->chars, "message") == 0) {
            msg_idx = i;
            break;
        }
    }
    if (msg_idx >= 0) {
        char* s = value_to_cstr(payload[msg_idx]);
        message = obj_string_new(ctx->gc, s, strlen(s));
        free(s);
    } else if (case_name) {
        message = case_name; /* permanent，可共享 */
    } else {
        message = obj_string_new(ctx->gc, "error", 5);
    }
    /* unsafe 窗口：message/payload 只被 C 局部引用，obj_error_new 内 gc_alloc 可能收集 */
    gc_begin_unsafe(ctx->gc);
    ObjError* e = obj_error_new(ctx->gc, message, set_name, case_name, pnames, payload, (size_t)n);
    R[instr_a(in)] = obj_error_val(e);
    gc_end_unsafe(ctx->gc);
}

static void vm_cold_error_info(mica_context* ctx, CallFrame* fr, Instr* pc, Instr in) {
    (void)in;
    fr->savedpc = pc;
    runtime_error(ctx, fr, "unexpected error descriptor");
}

static void vm_cold_is_error(mica_context* ctx, CallFrame* fr, Instr in) { (void)ctx;
    Value* R = fr->base;
    R[instr_a(in)] = val_bool(val_is_obj(R[instr_b(in)]) &&
                              val_as_obj(R[instr_b(in)])->type == OBJ_ERROR);
}

static void vm_cold_throw(mica_context* ctx, CallFrame* fr, Instr* pc, Instr in) {
    fr->savedpc = pc;
    Value* R = fr->base;
    const Value v = R[instr_a(in)];
    if (val_is_obj(v) && val_as_obj(v)->type == OBJ_ERROR) {
        ctx->st_error_value = v; /* Phase 2g：保存原错误对象（resume rethrow 保真） */
        runtime_error(ctx, fr, "%s", ((ObjError *)val_as_obj(v))->message->chars);
    } else {
        runtime_error(ctx, fr, "attempt to throw a non-error value");
    }
}

static void vm_cold_match_error(mica_context* ctx, CallFrame* fr, Instr in) { (void)ctx;
    const Proto* ch = fr->closure->proto;
    Value* R = fr->base;
    const Instr info = *fr->savedpc++;
    bool matched = false;
    const Value v = R[instr_b(in)];
    if (val_is_obj(v) && val_as_obj(v)->type == OBJ_ERROR) {
        ObjError* e = (ObjError *)val_as_obj(v);
        ObjString* set_pat = const_str_or_null(ch, instr_a(info));
        ObjString* case_pat = const_str_or_null(ch, instr_b(info));
        const bool set_ok = !set_pat || (e->set_name && obj_string_eq(e->set_name, set_pat));
        const bool case_ok = !case_pat || (e->case_name && obj_string_eq(e->case_name, case_pat));
        matched = set_ok && case_ok;
    }
    R[instr_a(in)] = val_bool(matched);
}

static void vm_cold_get_error_payload(mica_context* ctx, CallFrame* fr, Instr in) { (void)ctx;
    Value* R = fr->base;
    Value payload = val_null();
    const Value v = R[instr_b(in)];
    if (val_is_obj(v) && val_as_obj(v)->type == OBJ_ERROR) {
        ObjError* e = (ObjError *)val_as_obj(v);
        if (instr_c(in) >= 0 && (size_t)instr_c(in) < e->n_payload) {
            payload = e->payload[instr_c(in)];
        }
    }
    R[instr_a(in)] = payload;
}

static void vm_cold_new_enum_named(mica_context* ctx, CallFrame* fr, Instr in) {
    const Proto* ch = fr->closure->proto;
    Value* R = fr->base;
    const int n = instr_b(in);
    const int first = instr_c(in);
    const Instr info = *fr->savedpc++;
    if (instr_op(info) != OP_ENUM_INFO) {
        runtime_error(ctx, fr, "malformed enum descriptor");
        return;
    }
    ObjString* set_name = const_str_or_null(ch, instr_a(info));
    ObjString* case_name = const_str_or_null(ch, instr_b(info));
    Value* payload = NULL;
    if (n > 0) {
        payload = malloc((size_t)n * sizeof(Value));
        assert(payload != NULL);
        for (int i = 0; i < n; i++) payload[i] = R[first + i];
    }
    if (CTX_HAS_ERROR(ctx)) return;
    ObjEnumVal* e = obj_enum_val_new(ctx->gc, set_name, case_name, payload, (size_t)n);
    R[instr_a(in)] = obj_enum_val(e);
}

static void vm_cold_enum_info(mica_context* ctx, CallFrame* fr, Instr* pc, Instr in) {
    (void)in;
    fr->savedpc = pc;
    runtime_error(ctx, fr, "unexpected enum descriptor");
}

static void vm_cold_get_enum_payload(mica_context* ctx, CallFrame* fr, Instr in) { (void)ctx;
    Value* R = fr->base;
    Value payload = val_null();
    const Value v = R[instr_b(in)];
    if (val_is_obj(v) && val_as_obj(v)->type == OBJ_ENUM_VAL) {
        ObjEnumVal* e = (ObjEnumVal *)val_as_obj(v);
        if (instr_c(in) >= 0 && (size_t)instr_c(in) < e->n_payload) {
            payload = e->payload[instr_c(in)];
        }
    }
    R[instr_a(in)] = payload;
}

static void vm_cold_match_enum(mica_context* ctx, CallFrame* fr, Instr in) { (void)ctx;
    const Proto* ch = fr->closure->proto;
    Value* R = fr->base;
    const Instr info = *fr->savedpc++;
    bool matched = false;
    const Value v = R[instr_b(in)];
    if (val_is_obj(v) && val_as_obj(v)->type == OBJ_ENUM_VAL) {
        ObjEnumVal* e = (ObjEnumVal *)val_as_obj(v);
        ObjString* set_pat = const_str_or_null(ch, instr_a(info));
        ObjString* case_pat = const_str_or_null(ch, instr_b(info));
        const bool set_ok = !set_pat || (e->set_name && obj_string_eq(e->set_name, set_pat));
        const bool case_ok = !case_pat || (e->case_name && obj_string_eq(e->case_name, case_pat));
        matched = set_ok && case_ok;
    }
    R[instr_a(in)] = val_bool(matched);
}

static void vm_cold_match_literal(mica_context* ctx, CallFrame* fr, Instr in) { (void)ctx;
    const Proto* ch = fr->closure->proto;
    Value* R = fr->base;
    R[instr_a(in)] = val_bool(value_eq(R[instr_b(in)], ch->constants[instr_c(in)]));
}

static void vm_cold_is_type(mica_context* ctx, CallFrame* fr, Instr in) { (void)ctx;
    const Proto* ch = fr->closure->proto;
    Value* R = fr->base;
    const Value v = R[instr_b(in)];
    const Value d = ch->constants[instr_c(in)];
    bool r = vm_is_instance_of(v, d);
    if (val_is_string(d)) {
        r = value_matches_type_name(v, (ObjString *)val_as_obj(d));
    }
    R[instr_a(in)] = val_bool(r);
}

static void vm_cold_defer_mark(mica_context* ctx, CallFrame* fr, Instr in) {
    (void)fr;
    (void)in;
    defer_push(ctx, val_null());
}

static void vm_cold_defer_push(mica_context* ctx, CallFrame* fr, Instr in) {
    Value* R = fr->base;
    defer_push(ctx, R[instr_a(in)]);
}

static void vm_cold_defer_run(mica_context* ctx, CallFrame* fr, Instr* pc, Instr in) {
    (void)in;
    fr->savedpc = pc; /* 嵌套 vm_run 运行 defer 闭包后，主循环继续本帧需持久化恢复点 */
    CallFrame* cur = ctx_frame(ctx);
    const int mark = cur->defer_mark;
    size_t count = 0;
    for (int i = ctx->defer_top - 1; i >= mark; i--) {
        if (val_is_null(ctx->defers[i])) break;
        count++;
    }
    Value* defers = malloc((count ? count : 1) * sizeof(Value));
    size_t n = defer_collect(ctx, mark, defers, count);
    /* stop_count = 当前帧数：defer 闭包压一帧、返回后回到当前帧数 → 嵌套 vm_run
       在此停止，父帧剩余代码由本（主）vm_run 继续（勿用 frame_count-1，那会让嵌套
       vm_run 越界执行父帧 → while(true) 下 frame_count 归零后主循环崩）。 */
    defer_run_closures(ctx, defers, n, ctx->frame_count);
    free(defers);
}

/* ---- 长 opcode case 的冷路径外移（i-cache：热 case 保持紧凑，冷代码独立） ---- */

/* 未捕获错误检查：函数返回错误值且其调用未受 try/catch 保护 → 报错退出
   （throws 标注可选后，未被 catch 的错误直接终止程序）。 */
MICA_NOINLINE static void vm_check_uncaught_error(mica_context* ctx, CallFrame* cf, Instr* pc,
                                                  Value v) {
    if (MICA_LIKELY(!(cf->flags & CF_UNPROTECTED))) return;
    if (!val_is_obj(v) || val_as_obj(v)->type != OBJ_ERROR) return;
    ObjError* e = (ObjError *)val_as_obj(v);
    runtime_error_at(ctx, cf, pc, "unhandled error: %s",
                     e->message ? e->message->chars : "?");
}

/* 运行当前帧待运行的 defer（LIFO，同 OP_RETURN）。返回 true=出错（has_error）。 */
MICA_NOINLINE static bool vm_cold_run_defers(mica_context* ctx, CallFrame* cf, Instr* pc) {
    if (ctx->defer_top <= cf->defer_mark) return false;
    cf->savedpc = pc; /* 嵌套 vm_run 运行 defer 闭包后，主循环续本帧需持久化恢复点 */
    Value* defers = malloc((size_t)(ctx->defer_top - cf->defer_mark) * sizeof(Value));
    size_t n = defer_collect(ctx, cf->defer_mark, defers,
                             (size_t)(ctx->defer_top - cf->defer_mark));
    defer_run_closures(ctx, defers, n, ctx->frame_count);
    free(defers);
    return CTX_HAS_ERROR(ctx);
}

/* 冷路径：RETURN 的 defer 帧 / 构造器帧返回（与普通返回不同的帧处理）。
   返回 1=vm_run 应停止（frame_count 到 stop_count）；0=继续（调用方 VM_SYNC_FRAME）。 */
MICA_NOINLINE static int vm_cold_return_special(mica_context* ctx, CallFrame* cf, Instr* pc,
                                                Value* base, int stop_count) {
    cf->savedpc = pc; /* 特殊帧可能压 ctor-after / 跑 defer 嵌套 vm_run，持久化恢复点 */
    if (cf->flags & CF_DEFER) {
        /* defer 闭包帧返回：结果丢弃，defer_run_closures 负责链下一个 */
        obj_frame_close_capvals(ctx->gc, ctx, cf);
        for (int i = 0; i < (int)cf->closure->proto->max_regs; i++)
            base[i] = val_null();
        ctx_frame_pop(ctx);
        ctx->stack_top = base;
        return (ctx->frame_count <= stop_count) ? 1 : 0;
    }
    /* 构造器内部帧：不走普通返回计数；结果由实例决定 */
    obj_frame_close_capvals(ctx->gc, ctx, cf);
    ctx_frame_pop(ctx);
    /* 运行本帧 defer（构造器/init 内的 defer 须在帧退出时执行——否则延迟到外层作用域
       末尾，时机错误；与普通 RETURN 的 vm_cold_return_finish 一致）。出错 → 停止。 */
    if (MICA_UNLIKELY(vm_cold_run_defers(ctx, cf, pc))) return 1;
    if (cf->u.ctor.after) {
        /* defaults 帧返回 → 继续执行 init（同 base，self/args 原位） */
        ObjClosure* next = cf->u.ctor.after;
        const int cargc = cf->cargc;
        *base = obj_closure_val(next);
        base[1] = obj_instance_val(cf->u.ctor.constructed);
        for (int i = cargc + 1; i < (int)next->proto->arity; i++)
            base[1 + i] = val_null();
        if (!ensure_stack(ctx, base + (int)next->proto->max_regs)) {
            runtime_error(ctx, cf, "stack overflow (constructor init)");
            return 0;
        }
        base = cf->base; /* ensure_stack realloc 后重算（指针） */
        if (ctx->frame_count >= MICA_FRAME_MAX_SOFT) {
            runtime_error(ctx, cf, "stack overflow (too many frames)");
            return 0;
        }
        CallFrame nf = (CallFrame){
            .closure = next, .savedpc = next->proto->code, .u = { 0 }, .base = base,
            .want = RESULT_DISCARD, .defer_mark = ctx->defer_top,
        };
        nf.u.ctor.constructed = cf->u.ctor.constructed;
        nf.cargc = cf->cargc;
        nf.cwant = cf->cwant;
        *ctx_frame_new(ctx) = nf;
        ctx->stack_top = base + (int)next->proto->max_regs;
    } else {
        *base = obj_instance_val(cf->u.ctor.constructed);
        ctx->stack_top = (cf->cwant == RESULT_DISCARD) ? base : base + 1;
        for (int i = (cf->cwant == RESULT_DISCARD) ? 0 : 1;
             i < (int)cf->closure->proto->max_regs; i++)
            base[i] = val_null();
        if (ctx->frame_count <= stop_count) return 1;
    }
    return 0;
}

/* 冷路径：普通返回收尾——getter 结果写回父帧、清残留寄存器、运行 defer。返回 true=出错。 */
MICA_NOINLINE static bool vm_cold_return_finish(mica_context* ctx, CallFrame* cf, Instr* pc,
                                                Value* base, const Value* src, int count, bool is_getter) {
    cf->savedpc = pc;
    if (is_getter && cf->u.getter.getter_dest >= 0)
        cf->u.getter.getter_base[cf->u.getter.getter_dest] = *src;
    const int clear_from = is_getter ? 1 : count;
    for (int i = clear_from; i < (int)cf->closure->proto->max_regs; i++)
        base[i] = val_null();
    return vm_cold_run_defers(ctx, cf, pc);
}

/* 冷路径：EXIT 入口程序结束——清入口帧（关闭捕获值、清寄存器、运行 defer）、弹出。 */
MICA_NOINLINE static void vm_cold_exit(mica_context* ctx, CallFrame* fr, Instr* pc) {
    fr->savedpc = pc;
    Value* base = fr->base;
    obj_frame_close_capvals(ctx->gc, ctx, fr);
    for (int i = 0; i < (int)fr->closure->proto->max_regs; i++)
        base[i] = val_null();
    ctx_frame_pop(ctx);
    vm_cold_run_defers(ctx, fr, pc);
}

/* 属性 getter/setter 帧基址：必须压在父帧寄存器高水位之上（当前栈顶 与 父帧 max_regs 取大）。
   旧实现压 `fr->base + dest`（dest=GET_MEMBER 结果槽）——dest 可能是低槽局部变量（如 R0，
   `let d = self.prop` 把 d 放 R0），getter 帧与父帧寄存器重叠：getter 帧 base[1]=obj 写父帧 self，
   getter 返回时清 base[1..] 把父帧存活寄存器（self/临时）清成 null →
   "方法内访问计算属性后 self 变 null"。压到高水位之上，getter 帧与父帧零重叠（同 call_overload）。 */
static Value* prop_frame_base(mica_context* ctx, CallFrame* fr) {
    Value* high = fr->base + (int)fr->closure->proto->max_regs;
    return ctx->stack_top > high ? ctx->stack_top : high;
}

/* 冷路径：GET_MEMBER 属性 getter 压帧执行。返回 true=已压 getter 帧（调用方 VM_SYNC_FRAME）；
   false=无匹配 getter（调用方走 get_member）。 */
MICA_NOINLINE static bool vm_cold_get_member_getter(mica_context* ctx, CallFrame* fr, Instr* pc,
                                                    Instr in, Value obj, const String* name) {
    fr->savedpc = pc; /* 压 getter 帧前持久化父帧恢复点 */
    ObjClass* kl = ((ObjInstance *)val_as_obj(obj))->klass;
    Value gv = obj_class_find_getter_kv(kl, NAME_KV(name));
    if (val_is_null(gv)) return false;
    const int dest = instr_a(in);
    if (val_as_obj(gv)->type == OBJ_NATIVE_CLOSURE) { /* 原生属性 getter：直接调，结果写 dest 槽 */
        Value args[1] = { obj };
        Value r;
        if (!invoke_native_prop(ctx, (ObjNativeClosure *)val_as_obj(gv), args, 1, &r)) return true;
        fr->base[dest] = r;
        return true;
    }
    ObjClosure* g = (ObjClosure *)val_as_obj(gv);
    Value* new_base = prop_frame_base(ctx, fr);
    if (!ensure_stack(ctx, new_base + (int)g->proto->max_regs)) {
        runtime_error(ctx, fr, "stack overflow (getter)");
        return true;
    }
    new_base = prop_frame_base(ctx, fr); /* ensure_stack realloc 后重算（指针） */
    if (ctx->frame_count >= MICA_FRAME_MAX_SOFT) {
        runtime_error(ctx, fr, "stack overflow (too many frames)");
        return true;
    }
    *new_base = obj_closure_val(g);
    new_base[1] = obj;
    CallFrame nf = (CallFrame){
        .closure = g, .savedpc = g->proto->code, .u = { 0 }, .base = new_base,
        .flags = CF_GETTER, .want = RESULT_DISCARD, .defer_mark = ctx->defer_top,
    };
    nf.u.getter.getter_dest = dest;
    nf.u.getter.getter_base = fr->base;
    *ctx_frame_new(ctx) = nf;
    ctx->stack_top = new_base + (int)g->proto->max_regs;
    return true;
}

/* 冷路径：GET_MEMBER 类静态计算属性 getter 压帧执行（无 self，0 参）。 */
MICA_NOINLINE static bool vm_cold_get_member_class_getter(mica_context* ctx, CallFrame* fr,
                                                          Instr* pc, Instr in, ObjClass* kl,
                                                          const String* name) {
    fr->savedpc = pc; /* 压 getter 帧前持久化父帧恢复点 */
    Value gv = obj_class_find_getter_kv(kl, NAME_KV(name));
    if (val_is_null(gv)) return false;
    if (val_as_obj(gv)->type == OBJ_NATIVE_CLOSURE) return false; /* 原生属性是实例 getter（self），类访问不适用 */
    ObjClosure* g = (ObjClosure *)val_as_obj(gv);
    const int dest = instr_a(in);
    Value* new_base = prop_frame_base(ctx, fr);
    if (!ensure_stack(ctx, new_base + (int)g->proto->max_regs)) {
        runtime_error(ctx, fr, "stack overflow (getter)");
        return true;
    }
    new_base = prop_frame_base(ctx, fr); /* ensure_stack realloc 后重算（指针） */
    if (ctx->frame_count >= MICA_FRAME_MAX_SOFT) {
        runtime_error(ctx, fr, "stack overflow (too many frames)");
        return true;
    }
    *new_base = obj_closure_val(g);
    CallFrame nf = (CallFrame){
        .closure = g, .savedpc = g->proto->code, .u = { 0 }, .base = new_base,
        .flags = CF_GETTER, .want = RESULT_DISCARD, .defer_mark = ctx->defer_top,
    };
    nf.u.getter.getter_dest = dest;
    nf.u.getter.getter_base = fr->base;
    *ctx_frame_new(ctx) = nf;
    ctx->stack_top = new_base + (int)g->proto->max_regs;
    return true;
}

/* 调用点 IC 槽（D）：GET_MEMBER 指令位置 → IC 槽，per-proto 数组惰性 calloc（proto_free 释放）。
   类永久 + 方法经类可达 → 无 GC trace 需求。pc 指向当前指令之后（同 savedpc 语义）。 */
static CallSiteIC *vm_ic_slot(Proto *ch, const Instr *pc) {
    if (!ch->ics) {
        const size_t n = ch->count ? ch->count : 1;
        ch->ics = calloc(n, sizeof(CallSiteIC));
    }
    const size_t idx = (size_t)(pc - ch->code) - 1;
    return &ch->ics[idx];
}

/* 冷路径：SET_MEMBER 属性 setter 压帧执行。返回 true=已压 setter 帧；false=无匹配 setter。 */
MICA_NOINLINE static bool vm_cold_set_member_setter(mica_context* ctx, CallFrame* fr, Instr* pc,
                                                    Instr in, Value obj, Value val,
                                                    const String* name) {
    (void)in;
    fr->savedpc = pc; /* 压 setter 帧前持久化父帧恢复点 */
    ObjClass* klass = ((ObjInstance *)val_as_obj(obj))->klass;
    Value sv = obj_class_find_setter_kv(klass, NAME_KV(name));
    if (val_is_null(sv)) return false;
    if (val_as_obj(sv)->type == OBJ_NATIVE_CLOSURE) { /* 原生属性 setter：直接调（self, value） */
        Value args[2] = { obj, val };
        Value r;
        invoke_native_prop(ctx, (ObjNativeClosure *)val_as_obj(sv), args, 2, &r);
        return true;
    }
    ObjClosure* s = (ObjClosure *)val_as_obj(sv);
    Value* new_base = prop_frame_base(ctx, fr);
    if (!ensure_stack(ctx, new_base + (int)s->proto->max_regs)) {
        runtime_error(ctx, fr, "stack overflow (setter)");
        return true;
    }
    new_base = prop_frame_base(ctx, fr); /* ensure_stack realloc 后重算（指针） */
    if (ctx->frame_count >= MICA_FRAME_MAX_SOFT) {
        runtime_error(ctx, fr, "stack overflow (too many frames)");
        return true;
    }
    *new_base = obj_closure_val(s);
    new_base[1] = obj;
    new_base[2] = val;
    CallFrame nf = (CallFrame){
        .closure = s, .savedpc = s->proto->code, .u = { 0 }, .base = new_base,
        .flags = CF_GETTER, .want = RESULT_DISCARD, .defer_mark = ctx->defer_top,
    };
    nf.u.getter.getter_dest = -1;
    nf.u.getter.getter_base = fr->base;
    *ctx_frame_new(ctx) = nf;
    ctx->stack_top = new_base + (int)s->proto->max_regs;
    return true;
}

/* 冷路径：SET_MEMBER IC miss——setter/只读/字段解析回填/报错。i-cache 外移（case 只留
   IC 命中快路径）。IC 命中即确认无 setter/只读（类定义编译期固定，klass 不变则不变）。 */
MICA_NOINLINE static void vm_cold_set_member_miss(mica_context* ctx, CallFrame* fr, Instr* pc,
                                                  Instr in, Value obj, Value val,
                                                  const String* name) {
    fr->savedpc = pc;
    ObjClass* klass = ((ObjInstance *)val_as_obj(obj))->klass;
    CallSiteIC* ic = vm_ic_slot((Proto *)fr->closure->proto, pc);
    /* 属性 setter（冷）：实例 + 类有 setter → 压 setter 帧执行（self, value） */
    if (MICA_UNLIKELY(klass->has_setters)) {
        if (vm_cold_set_member_setter(ctx, fr, pc, in, obj, val, name)) return;
    }
    /* 只有 getter 无 setter：只读属性赋值报错（避免被动态字段遮蔽） */
    if (MICA_UNLIKELY(klass->has_getters) && !val_is_null(obj_class_find_getter_kv(klass, NAME_KV(name)))) {
        runtime_error(ctx, fr, "property '%s' is read-only", str_data(name));
        return;
    }
    const int idx = obj_class_field_index(klass, name);
    if (idx >= 0) {
        ic->klass = klass; ic->kind = 1; ic->field_idx = (int16_t)idx;
        ((ObjInstance *)val_as_obj(obj))->fields[idx] = val;
        if (val_is_obj(val)) gc_record_write(ctx->gc, (Obj *)val_as_obj(obj), val);
        return;
    }
    runtime_error(ctx, fr, "instance has no field '%s'", str_data(name));
}

/* 冷路径：TAILCALL 复用当前帧——实参下移到帧基址、补默认参、重置 closure/savedpc/
   argc/flags/u/defer_mark（base/want 保持，被调用函数结果直接透传给本帧调用者）。
   深递归每层付一次 C 调用（仍远快于普通调用的压帧/弹帧）。

   ⚠️ **当前停用**（无调用点）：实参搬运与 vm_call_dispatch 的槽布局约定不一致，见
   VM_CASE(TAILCALL) 的说明。保留函数体以便对齐后重新启用；用 #if 0 消除未使用告警。 */
#if 0
MICA_NOINLINE static void vm_cold_tailcall_reuse(mica_context* ctx, CallFrame* cf, Value fn,
                                                 ObjClosure* cl, Proto* f, int a, int argc,
                                                 uint8_t keep_flags) {
    /* 帧复用前先关闭旧闭包的开放捕获值：旧闭包外逃 lambda 的 open capval 指向
       [base, base+旧 max_regs) 栈槽，下面 memmove/新函数执行会覆盖这些槽——不关闭
       则外逃闭包读到被调函数的值（正常 RETURN 路径都先 close，TAILCALL 曾漏）。 */
    if (MICA_UNLIKELY(ctx->opencapval != NULL)) obj_frame_close_capvals(ctx->gc, ctx, cf);
    /* ⚠️ 顺序：**先** ensure_stack 再写默认参数槽。原实现先 base[1+i] 填 null（i 到 arity-1）
       再做 ensure_stack，而 ensure_stack 保护的边界只到 base+max_regs —— arity 可大于
       max_regs 的情形下填充就越界写（栈块末尾之外）。 */
    const ptrdiff_t base_off = (ptrdiff_t)(cf->base - ctx->stack);
    const int need = (int)f->max_regs > (int)f->arity + 1 ? (int)f->max_regs : (int)f->arity + 1;
    if (!ensure_stack(ctx, ctx->stack + base_off + need)) {
        runtime_error(ctx, cf, "stack overflow (call too deep)");
        return;
    }
    Value* base = ctx->stack + base_off; /* ensure_stack realloc 后重算（指针） */
    /* 实参搬运（注意 codegen 的槽约定）：
       codegen 为被调函数准备的是 [base+a]=函数值、[base+a+1]=self（普通函数为 null）、
       [base+a+2..] = 用户实参；argc **含 self 槽**，即用户实参个数 = argc-1。
       帧复用后新函数的布局必须是 [base]=函数值、[base+1]=self、[base+2..]=用户实参。
       因此只需把用户实参从 a+2 起搬回 1 起（左移 1），**self 槽原地不动**——
       因为 a+1 的 self 正好等于新布局要求的 base+1。
       旧实现按 `memmove(base[1], base[a+1], argc)` 搬运，把 self 空洞一起搬进来并
       覆盖掉最后一个实参（实测 `func tick(p){return p>0}` + `return tick(prog)`
       的 p 取到 null）——这是帧复用长期失效的根因。 */
    const int nargs = argc - 1; /* 用户实参个数（不含 self 槽） */
    if (a > 0 && nargs > 0) {
        memmove(&base[2], &base[a + 2], (size_t)nargs * sizeof(Value));
    }
    *base = fn;
    base[1] = val_null(); /* self 槽：普通函数 null；方法调用时由 codegen 已排在 a+1，见上 */
    for (int i = nargs; i < (int)f->arity; i++) base[2 + i] = val_null(); /* 补默认参 */
    cf->base = base;
    cf->closure = cl;
    cf->savedpc = f->code;
    cf->argc = (uint16_t)argc;
    /* 保留调用点的未保护标记：旧实现置 0 会吞掉 return g() 抛出的未捕获错误值
       （CF_UNPROTECTED 是"该调用未受 try/catch 保护"的语义，帧复用不改变它）。 */
    cf->flags = keep_flags;
    memset(&cf->u, 0, sizeof(cf->u));
    cf->defer_mark = ctx->defer_top;
    ctx->stack_top = base + (int)f->max_regs;
}
#endif /* 0：TAILCALL 帧复用停用（槽布局不一致，见 VM_CASE(TAILCALL) 说明） */

static void vm_cold_closure(mica_context* ctx, CallFrame* fr, Instr in) {
    const Proto* ch = fr->closure->proto;
    Value* R = fr->base;
    /* 捕获来源查 proto->capvals（编译期落表）：
       is_local=当前帧局部槽 → find-or-create 开放捕获值指向该槽；
       !is_local=外层闭包捕获值 → 直接共享（传递捕获）。 */
    ObjClosure* pc = (ObjClosure *)val_as_obj(ch->constants[instr_b(in)]);
    ObjClosure* cl = obj_closure_new(ctx->gc, pc->proto, (uint8_t)instr_c(in));
    R[instr_a(in)] = obj_closure_val(cl); /* 立即入根 */
    for (int i = 0; i < instr_c(in); i++) {
        const CapvalDesc* d = &pc->proto->capvals[i];
        cl->capvals[i] = d->is_local
                             ? frame_capval_for(ctx, fr, d->index)
                             : fr->closure->capvals[d->index];
    }
}

static void vm_cold_get_capval(mica_context* ctx, CallFrame* fr, Instr in) { (void)ctx;
    Value* R = fr->base;
    ObjCapVal* cap = fr->closure->capvals[instr_b(in)];
    R[instr_a(in)] = obj_capval_get(cap);
}

static void vm_cold_set_capval(mica_context* ctx, CallFrame* fr, Instr in) {
    Value* R = fr->base;
    ObjCapVal* cap = fr->closure->capvals[instr_b(in)];
    obj_capval_set(ctx->gc, cap, R[instr_a(in)]);
}

static void vm_cold_varargs(mica_context* ctx, CallFrame* fr, Instr in) {
    Value* R = fr->base;
    /* vararg：把当前帧超出声明参数（arity）的多余实参收集成数组。
       注意：目标寄存器可能与实参槽重叠，必须先收集完再写 R[dest]。 */
    const int extra = fr->argc - (int)fr->closure->proto->arity;
    ObjArray* arr = obj_array_new(ctx->gc, extra > 0 ? (size_t)extra : 0);
    for (int i = 0; i < extra; i++) {
        obj_array_push(ctx->gc, arr,
                       fr->base[1 + (int)fr->closure->proto->arity + i]);
    }
    R[instr_a(in)] = obj_array_val(arr);
}

static void vm_cold_get_method(mica_context* ctx, CallFrame* fr, Instr* pc, Instr in) {
    fr->savedpc = pc;
    const Proto* ch = fr->closure->proto;
    Value* R = fr->base;
    Value cv = R[instr_c(in)];
    if (!(val_is_obj(cv) && val_as_obj(cv)->type == OBJ_CLASS)) {
        runtime_error(ctx, fr, "OP_GET_METHOD on non-class");
        return;
    }
    const String* mname = &ch->ident_names[instr_b(in)];
    ObjClosure* m = obj_class_find_method_kv((ObjClass *)val_as_obj(cv),
                                             mname->hash, str_data(mname), str_len(mname));
    if (!m) {
        runtime_error(ctx, fr, "class has no method '%s'", str_data(mname));
        return;
    }
    R[instr_a(in)] = obj_closure_val(m);
}

static void vm_cold_array(mica_context* ctx, CallFrame* fr, Instr in) {
    Value* R = fr->base;
    const int n = instr_b(in);
    const int first = instr_c(in);
    ObjArray* arr = obj_array_new(ctx->gc, (size_t)n);
    R[instr_a(in)] = obj_array_val(arr); /* 立即入根 */
    for (int i = 0; i < n; i++) obj_array_push(ctx->gc, arr, R[first + i]);
}

static void vm_cold_map(mica_context* ctx, CallFrame* fr, Instr in) {
    Value* R = fr->base;
    const int n = instr_b(in);
    const int first = instr_c(in);
    ObjMap* m = obj_map_new(ctx->gc);
    R[instr_a(in)] = obj_map_val(m); /* 立即入根 */
    for (int i = 0; i < n; i++) {
        obj_map_set(ctx->gc, m, R[first + 2 * i], R[first + 2 * i + 1]);
    }
}

/* vm_run 帧缓存同步（Phase 1.2）：可能改帧（frame_count）或 realloc 栈（ensure_stack）
   的指令后，重新装载当前帧的 fr/ch/R，并重载局部指令指针 pc（从新顶帧的 savedpc 读，
   只读不写——TAILCALL-reuse 后 cf->savedpc=f->code 正是新函数起点）。漏同步 = 悬垂指针 UAF。
   热分派源是局部 pc（`in = *pc++`），fr->savedpc 只在帧边界/挂起点写回（VM_SAVE_PC）。 */
#define VM_SAVE_PC() do { fr->savedpc = pc; } while (0)
/* 重载帧缓存（fr/ch/R/pc）。同时保证新 ch 的索引化全局已解析（global_entries 非 NULL）——
   把 LOAD_GLOBAL/STORE_GLOBAL/GLOAD_INDEX 热路径的 `!ch->global_entries` 惰性检查前置到
   每次帧建立一次（替代每条全局指令一次；bench_game 全局指令 151 条 >> 帧建立次数）。
   安全性：vm_run 内 ch 仅在函数开头与本宏赋值，故覆盖完备（无遗漏路径）。 */
#define VM_SYNC_FRAME() do { \
        fr = ctx_frame(ctx); \
        ch = fr->closure->proto; \
        R = fr->base; \
        pc = fr->savedpc; \
    } while (0)
/* 冷 helper 消费 savedpc 数据字（NEW_ERROR_NAMED/MATCH_ERROR/GET_ERROR_PAYLOAD/MATCH_ENUM
   内部 `*fr->savedpc++`）：save 前置（fr->savedpc=pc 指向当前指令之后） + resync 后置
   （helper 推进了 savedpc，局部 pc 同步到其值）。仅用于这四个 info helper。 */
#define VM_CALL_COLD(call) do { VM_SAVE_PC(); call; pc = fr->savedpc; } while (0)

CoroResult vm_run(mica_context* ctx, Value* out, int stop_count) {
    /* 错误已置位（mica_call 直调 native 的错误走标志式返回后转入 vm_run）：入口拦截，
       防带标志进入循环。正常路径 mica_run/mica_call 进 vm_run 前已清标志。 */
    if (CTX_HAS_ERROR(ctx)) return CORO_ERROR;
    /* 错误跳转缓冲：runtime_error 在 active_jmp 有效时 longjmp 到此，跳过主循环顶部的
       每迭代 has_error 检查（Lua 式错误抛出）。嵌套 vm_run 各自 save/restore，LIFO 正确；
       setjmp 非零分支只返回，不读被 longjmp 污染的局部变量（prev_jmp/ctx 其后未改，安全）。 */
    jmp_buf jb;
    jmp_buf* prev_jmp = ctx->active_jmp;
    ctx->active_jmp = &jb;
    if (setjmp(jb) != 0) { /* runtime_error longjmp 落点 */
        ctx->active_jmp = prev_jmp;
        return CORO_ERROR;
    }
    /* EXIT/RETURN 把帧数压到 stop_count 时置 done 跳出，替代每迭代一次的
       `frame_count > stop_count` 比较（热点循环省一次 cmp+branch）。 */
    bool done = false;
    /* 帧循环不变量（Phase 1.2）：fr/ch/R 提升为循环级缓存——纯寄存器指令
       （MOV/算术/比较/跳转/常量）零 reload，每迭代省约 5 次内存 load；
       只在可能改帧/栈的指令（CALL/RETURN/INDEX/INDEX_SET/GET/SET_MEMBER getter-setter/
       DEFER_RUN）后经 VM_SYNC_FRAME 重新装载。指令取自分派热路径的局部 `*pc++`
       （纯寄存器，fr->savedpc 不每迭代存取）；跳转写局部 pc。 */
    CallFrame* fr = ctx_frame(ctx);
    Proto* ch = fr->closure->proto;
    Value* R = fr->base;
    Instr* pc = fr->savedpc; /* 指令指针（热分派源）：指向"下一条"指令；只在帧边界/挂起点写回 fr->savedpc */
    /* 循环级缓存 rt->globals（如 fr/ch/R）：LOAD_GLOBAL/STORE_GLOBAL/GLOAD_INDEX 的热路径
       每次 ctx_globals(ctx) = ctx->rt->globals 两跳解引用，合计 151 条/程序（bench_game）；
       缓存为局部指针后省两跳（rt->globals 运行期不变，安全）。 */
    ObjTable* const globals = ctx_globals(ctx);
    Instr in; /* 当前指令（两模式共享：switch 在 while 内赋值，CG 在 VM_DISPATCH 内赋值） */
#if VM_COMPUTED_GOTO
    /* 直接线程化分发表：8 位 opcode 全覆盖，未定义 op → L_VM_DEFAULT（兜底报 unknown opcode）。
       `[0...255]=&&L_VM_DEFAULT` 范围默认 + `[OP_X]=&&L_VM_OP_X` 单个覆盖是 GNU 指定器预期
       行为（clang -Winitializer-overrides 提示为良性，此处抑制）。 */
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Winitializer-overrides"
#endif
    static void* const k_vm_targets[256] = {
        [0 ... 255] = &&L_VM_DEFAULT,
        [OP_MOV] = &&L_VM_OP_MOV,
        [OP_LOAD_GLOBAL] = &&L_VM_OP_LOAD_GLOBAL,
        [OP_STORE_GLOBAL] = &&L_VM_OP_STORE_GLOBAL,
        [OP_INDEX] = &&L_VM_OP_INDEX,
        [OP_INDEX_SET] = &&L_VM_OP_INDEX_SET,
        [OP_CONST] = &&L_VM_OP_CONST,
        [OP_KNUM] = &&L_VM_OP_KNUM,
        [OP_KSTR] = &&L_VM_OP_KSTR,
        [OP_MOVI] = &&L_VM_OP_MOVI,
        [OP_NULL] = &&L_VM_OP_NULL,
        [OP_TRUE] = &&L_VM_OP_TRUE,
        [OP_FALSE] = &&L_VM_OP_FALSE,
        [OP_ADD] = &&L_VM_OP_ADD,
        [OP_SUB] = &&L_VM_OP_SUB,
        [OP_MUL] = &&L_VM_OP_MUL,
        [OP_DIV] = &&L_VM_OP_DIV,
        [OP_IDIV] = &&L_VM_OP_IDIV,
        [OP_MOD] = &&L_VM_OP_MOD,
        [OP_POW] = &&L_VM_OP_POW,
        [OP_NEG] = &&L_VM_OP_NEG,
        [OP_NOT] = &&L_VM_OP_NOT,
        [OP_ADDVN] = &&L_VM_OP_ADDVN,
        [OP_SUBVN] = &&L_VM_OP_SUBVN,
        [OP_MULVN] = &&L_VM_OP_MULVN,
        [OP_DIVVN] = &&L_VM_OP_DIVVN,
        [OP_IDIVVN] = &&L_VM_OP_IDIVVN,
        [OP_MODVN] = &&L_VM_OP_MODVN,
        [OP_ADDNV] = &&L_VM_OP_ADDNV,
        [OP_MULNV] = &&L_VM_OP_MULNV,
        [OP_INC] = &&L_VM_OP_INC,
        [OP_DEC] = &&L_VM_OP_DEC,
        [OP_EQ] = &&L_VM_OP_EQ,
        [OP_NE] = &&L_VM_OP_NE,
        [OP_LT] = &&L_VM_OP_LT,
        [OP_LE] = &&L_VM_OP_LE,
        [OP_ISEQ] = &&L_VM_OP_ISEQ,
        [OP_ISNE] = &&L_VM_OP_ISNE,
        [OP_ISLT] = &&L_VM_OP_ISLT,
        [OP_ISLE] = &&L_VM_OP_ISLE,
        [OP_ISLT_LEN] = &&L_VM_OP_ISLT_LEN,
        [OP_GET_MEMBER] = &&L_VM_OP_GET_MEMBER,
        [OP_SET_MEMBER] = &&L_VM_OP_SET_MEMBER,
        [OP_LENGTH] = &&L_VM_OP_LENGTH,
        [OP_JMP] = &&L_VM_OP_JMP,
        [OP_JMP_IF_FALSE] = &&L_VM_OP_JMP_IF_FALSE,
        [OP_JMP_IF_TRUE] = &&L_VM_OP_JMP_IF_TRUE,
        [OP_FORI] = &&L_VM_OP_FORI,
        [OP_FORL] = &&L_VM_OP_FORL,
        [OP_CALL] = &&L_VM_OP_CALL,
        [OP_FASTCALL] = &&L_VM_OP_FASTCALL,
        [OP_RET0] = &&L_VM_OP_RET0,
        [OP_RET1] = &&L_VM_OP_RET1,
        [OP_RETURN] = &&L_VM_OP_RETURN,
        [OP_CLOSURE] = &&L_VM_OP_CLOSURE,
        [OP_GET_CAPVAL] = &&L_VM_OP_GET_CAPVAL,
        [OP_SET_CAPVAL] = &&L_VM_OP_SET_CAPVAL,
        [OP_VARARGS] = &&L_VM_OP_VARARGS,
        [OP_CONCAT] = &&L_VM_OP_CONCAT,
        [OP_GET_METHOD] = &&L_VM_OP_GET_METHOD,
        [OP_ARRAY] = &&L_VM_OP_ARRAY,
        [OP_MAP] = &&L_VM_OP_MAP,
        [OP_ERROR] = &&L_VM_OP_ERROR,
        [OP_NEW_ERROR] = &&L_VM_OP_NEW_ERROR,
        [OP_NEW_ERROR_NAMED] = &&L_VM_OP_NEW_ERROR_NAMED,
        [OP_ERROR_INFO] = &&L_VM_OP_ERROR_INFO,
        [OP_IS_ERROR] = &&L_VM_OP_IS_ERROR,
        [OP_THROW] = &&L_VM_OP_THROW,
        [OP_MATCH_ERROR] = &&L_VM_OP_MATCH_ERROR,
        [OP_GET_ERROR_PAYLOAD] = &&L_VM_OP_GET_ERROR_PAYLOAD,
        [OP_NEW_ENUM_NAMED] = &&L_VM_OP_NEW_ENUM_NAMED,
        [OP_ENUM_INFO] = &&L_VM_OP_ENUM_INFO,
        [OP_GET_ENUM_PAYLOAD] = &&L_VM_OP_GET_ENUM_PAYLOAD,
        [OP_MATCH_ENUM] = &&L_VM_OP_MATCH_ENUM,
        [OP_MATCH_LITERAL] = &&L_VM_OP_MATCH_LITERAL,
        [OP_IS_TYPE] = &&L_VM_OP_IS_TYPE,
        [OP_YIELD] = &&L_VM_OP_YIELD,
        [OP_DEFER_MARK] = &&L_VM_OP_DEFER_MARK,
        [OP_DEFER_PUSH] = &&L_VM_OP_DEFER_PUSH,
        [OP_DEFER_RUN] = &&L_VM_OP_DEFER_RUN,
        [OP_EXIT] = &&L_VM_OP_EXIT,
        [OP_TAILCALL] = &&L_VM_OP_TAILCALL,
        [OP_GLOAD_INDEX] = &&L_VM_OP_GLOAD_INDEX,
        [OP_ISEQN] = &&L_VM_OP_ISEQN,
        [OP_ISNEN] = &&L_VM_OP_ISNEN,
        [OP_ISEQS] = &&L_VM_OP_ISEQS,
        [OP_ISNES] = &&L_VM_OP_ISNES,
        [OP_GETI] = &&L_VM_OP_GETI,
        [OP_SETI] = &&L_VM_OP_SETI,
        [OP_ADDI] = &&L_VM_OP_ADDI,
        [OP_SUBI] = &&L_VM_OP_SUBI,
    };
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
    VM_DISPATCH(); /* 首条指令分派 */
#else
    while (true) {
        in = *pc++;
        switch (instr_op(in)) {
#endif
        VM_CASE(MOV) {
            R[instr_a(in)] = R[instr_b(in)];
            VM_DISPATCH();
        }
        VM_CASE(LOAD_GLOBAL) {
            /* 稠密下标直读（无版本守卫）：下标在表 rehash / dense 增长后稳定。
               -1 = 未定义：惰性重查（先引用后定义的全局，定义后首访缓存生效）→ 仍无则报错。 */
            if (MICA_UNLIKELY(!ch->global_entries)) proto_resolve_globals(globals, ch);
            const int32_t idx = ch->global_entries[instr_bx(in)];
            if (MICA_UNLIKELY(idx < 0)) {
                const String* name = &ch->global_ident_names[instr_bx(in)];
                const size_t d = objtable_find_dense(globals, name->hash, str_data(name),
                                                     str_len(name));
                if (d == TABLE_NULL_ELEM) {
                    char nbuf[520];
                    runtime_error_at(ctx, fr, pc, "undefined global '%s'",
                                     ident_display(name, nbuf, sizeof(nbuf)));
                    VM_DISPATCH();
                }
                ch->global_entries[instr_bx(in)] = (int32_t)d;
                R[instr_a_ad(in)] = globals->values[d];
                VM_DISPATCH();
            }
            R[instr_a_ad(in)] = globals->values[idx];
            VM_DISPATCH();
        }
        VM_CASE(STORE_GLOBAL) {
            if (MICA_UNLIKELY(!ch->global_entries)) proto_resolve_globals(globals, ch);
            const int32_t idx = ch->global_entries[instr_bx(in)];
            const Value v = R[instr_a_ad(in)];
            if (MICA_UNLIKELY(idx < 0)) { /* 新全局首次定义：objtable_set_kv 创建（可能 rehash/dense 增长）→ 缓存下标 */
                const String* name = &ch->global_ident_names[instr_bx(in)];
                objtable_set_kv(ctx->gc, globals, name->hash, str_data(name),
                                str_len(name), v);
                ch->global_entries[instr_bx(in)] =
                        (int32_t)objtable_find_dense(globals, name->hash, str_data(name),
                                                     str_len(name));
            } else {
                globals->values[idx] = v;
                if (val_is_obj(v)) gc_record_write(ctx->gc, (Obj *)globals, v);
            }
            VM_DISPATCH();
        }
        VM_CASE(INDEX) {
            const Value obj = R[instr_b(in)];
            if (MICA_LIKELY(MICA_TVISARRAY(obj))) { /* itype 单比较（LuaJIT 风格） */
                const Value idx = R[instr_c(in)];
                if (MICA_LIKELY(MICA_TVISNUM(idx))) {
                    const ObjArray* a = (ObjArray *)val_as_obj(obj);
                    int64_t i;
                    if (MICA_UNLIKELY(!array_index_of(ctx, fr, pc, a, idx, &i))) VM_DISPATCH();
                    R[instr_a(in)] = a->items[i]; /* 数组读快路径（Phase 2） */
                    VM_DISPATCH();
                }
            }
            const Value r = index_get(ctx, fr, pc, obj, R[instr_c(in)]);
            VM_SYNC_FRAME(); /* index_get 运算符重载可能压帧 + realloc 栈；先取结果再写 R */
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(INDEX_SET) {
            const Value obj = R[instr_b(in)];
            if (MICA_LIKELY(MICA_TVISARRAY(obj))) {
                const Value idx = R[instr_c(in)];
                if (MICA_LIKELY(MICA_TVISNUM(idx))) {
                    ObjArray* a = (ObjArray *)val_as_obj(obj);
                    int64_t i;
                    if (MICA_UNLIKELY(!array_index_of(ctx, fr, pc, a, idx, &i))) VM_DISPATCH();
                    const Value v = R[instr_a(in)];
                    a->items[i] = v;
                    obj_array_update_kind(a, v);
                    if (val_is_obj(v)) gc_record_write(ctx->gc, (Obj *)a, v); /* 写标量免写屏障 */
                    VM_DISPATCH();
                }
            }
            index_set(ctx, fr, pc, obj, R[instr_c(in)], R[instr_a(in)]);
            VM_SYNC_FRAME(); /* index_set 运算符重载可能压帧 + realloc 栈 */
            VM_DISPATCH();
        }
        VM_CASE(GLOAD_INDEX) {
            if (MICA_UNLIKELY(!ch->global_entries)) proto_resolve_globals(globals, ch);
            const int32_t gidx = ch->global_entries[instr_b(in)];
            Value obj;
            if (MICA_UNLIKELY(gidx < 0)) { /* 未定义：惰性重查（先引用后定义，定义后首访缓存生效） */
                const String* name = &ch->global_ident_names[instr_b(in)];
                const size_t d = objtable_find_dense(globals, name->hash, str_data(name),
                                                     str_len(name));
                if (d == TABLE_NULL_ELEM) {
                    char nbuf[520];
                    runtime_error_at(ctx, fr, pc, "undefined global '%s'",
                                     ident_display(name, nbuf, sizeof(nbuf)));
                    VM_DISPATCH();
                }
                ch->global_entries[instr_b(in)] = (int32_t)d;
                obj = globals->values[d];
            } else {
                obj = globals->values[gidx];
            }
            if (MICA_LIKELY(MICA_TVISARRAY(obj))) {
                const Value idx = R[instr_c(in)];
                if (MICA_LIKELY(MICA_TVISNUM(idx))) {
                    const ObjArray* a = (ObjArray *)val_as_obj(obj);
                    int64_t i;
                    if (MICA_UNLIKELY(!array_index_of(ctx, fr, pc, a, idx, &i))) VM_DISPATCH();
                    R[instr_a(in)] = a->items[i];
                    VM_DISPATCH();
                }
            }
            const Value r = index_get(ctx, fr, pc, obj, R[instr_c(in)]);
            VM_SYNC_FRAME(); /* index_get 运算符重载可能压帧 + realloc 栈；先取结果再写 R */
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(ADDI) { /* R[a] = R[b] + imm（有符号 8 位立即数，免常量池+免加载） */
            const Value vb = R[instr_b(in)];
            if (MICA_LIKELY(val_is_num(vb))) {
                R[instr_a(in)] = val_float(val_as_float(vb) + (double)(int8_t)instr_c(in));
                VM_DISPATCH();
            }
            const Value r = op_add_slow(ctx, fr, pc, vb, val_float((double)(int8_t)instr_c(in)));
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(SUBI) { /* R[a] = R[b] - imm */
            const Value vb = R[instr_b(in)];
            if (MICA_LIKELY(val_is_num(vb))) {
                R[instr_a(in)] = val_float(val_as_float(vb) - (double)(int8_t)instr_c(in));
                VM_DISPATCH();
            }
            const Value r = op_sub_slow(ctx, fr, pc, vb, val_float((double)(int8_t)instr_c(in)));
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(GETI) { /* R[a] = R[b][常量下标]（ABC，c=num 池常量下标 8 位） */
            const Value obj = R[instr_b(in)];
            if (MICA_LIKELY(MICA_TVISARRAY(obj))) {
                const ObjArray* a = (ObjArray *)val_as_obj(obj);
                int64_t i;
                if (MICA_UNLIKELY(!array_index_of(ctx, fr, pc, a, ch->num_consts[instr_c(in)], &i)))
                    VM_DISPATCH();
                R[instr_a(in)] = a->items[i];
                VM_DISPATCH();
            }
            const Value r = index_get(ctx, fr, pc, obj, ch->num_consts[instr_c(in)]);
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(SETI) { /* R[b][常量下标] = R[a] */
            const Value obj = R[instr_b(in)];
            if (MICA_LIKELY(MICA_TVISARRAY(obj))) {
                ObjArray* a = (ObjArray *)val_as_obj(obj);
                int64_t i;
                if (MICA_UNLIKELY(!array_index_of(ctx, fr, pc, a, ch->num_consts[instr_c(in)], &i)))
                    VM_DISPATCH();
                const Value v = R[instr_a(in)];
                a->items[i] = v;
                obj_array_update_kind(a, v);
                if (val_is_obj(v)) gc_record_write(ctx->gc, (Obj *)a, v);
                VM_DISPATCH();
            }
            index_set(ctx, fr, pc, obj, ch->num_consts[instr_c(in)], R[instr_a(in)]);
            VM_SYNC_FRAME();
            VM_DISPATCH();
        }
        VM_CASE(CONST) {
            R[instr_a_ad(in)] = ch->constants[instr_bx(in)];
            VM_DISPATCH();
        }
        VM_CASE(KNUM) {
            R[instr_a_ad(in)] = ch->num_consts[instr_bx(in)];
            VM_DISPATCH();
        }
        VM_CASE(KSTR) {
            R[instr_a_ad(in)] = ch->str_consts[instr_bx(in)];
            VM_DISPATCH();
        }
        VM_CASE(MOVI) {
            R[instr_a_ad(in)] = val_int(instr_sbx(in));
            VM_DISPATCH();
        }
        VM_CASE(NULL) {
            R[instr_a(in)] = val_null();
            VM_DISPATCH();
        }
        VM_CASE(TRUE) {
            R[instr_a(in)] = val_true();
            VM_DISPATCH();
        }
        VM_CASE(FALSE) {
            R[instr_a(in)] = val_false();
            VM_DISPATCH();
        }
        VM_CASE(ADD) {
            const Value vb = R[instr_b(in)], vc = R[instr_c(in)];
            if (MICA_LIKELY(val_both_num(vb, vc))) {
                R[instr_a(in)] = val_float(val_as_float(vb) + val_as_float(vc));
                VM_DISPATCH();
            }
            const Value r = op_add_slow(ctx, fr, pc, vb, vc);
            VM_SYNC_FRAME(); /* 重载压帧 / 拼接分配可能 realloc 栈 */
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(SUB) {
            const Value vb = R[instr_b(in)], vc = R[instr_c(in)];
            if (MICA_LIKELY(val_both_num(vb, vc))) {
                R[instr_a(in)] = val_float(val_as_float(vb) - val_as_float(vc));
                VM_DISPATCH();
            }
            const Value r = op_sub_slow(ctx, fr, pc, vb, vc);
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(MUL) {
            const Value vb = R[instr_b(in)], vc = R[instr_c(in)];
            if (MICA_LIKELY(val_both_num(vb, vc))) {
                R[instr_a(in)] = val_float(val_as_float(vb) * val_as_float(vc));
                VM_DISPATCH();
            }
            const Value r = op_mul_slow(ctx, fr, pc, vb, vc);
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(DIV) {
            const Value vb = R[instr_b(in)], vc = R[instr_c(in)];
            if (MICA_LIKELY(val_both_num(vb, vc))) {
                const double db = val_as_float(vc);
                if (MICA_LIKELY(db != 0)) {
                    R[instr_a(in)] = val_float(val_as_float(vb) / db);
                    VM_DISPATCH();
                }
            }
            const Value r = op_div_slow(ctx, fr, pc, vb, vc);
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(IDIV) {
            const Value vb = R[instr_b(in)], vc = R[instr_c(in)];
            if (MICA_LIKELY(val_both_num(vb, vc))) {
                const double db = val_as_float(vc);
                if (MICA_LIKELY(db != 0)) {
                    R[instr_a(in)] = val_float(floor(val_as_float(vb) / db));
                    VM_DISPATCH();
                }
            }
            const Value r = op_idiv_slow(ctx, fr, pc, vb, vc);
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(MOD) {
            const Value vb = R[instr_b(in)], vc = R[instr_c(in)];
            if (MICA_LIKELY(val_both_num(vb, vc))) {
                const double db = val_as_float(vc);
                if (MICA_LIKELY(db != 0)) {
                    double rm = fmod(val_as_float(vb), db);
                    if (rm != 0 && ((rm < 0) != (db < 0))) rm += db;
                    R[instr_a(in)] = val_float(rm);
                    VM_DISPATCH();
                }
            }
            const Value r = op_mod_slow(ctx, fr, pc, vb, vc);
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(POW) {
            const Value vb = R[instr_b(in)], vc = R[instr_c(in)];
            if (MICA_LIKELY(val_both_num(vb, vc))) {
                const double d = pow(val_as_float(vb), val_as_float(vc));
                if (MICA_LIKELY(!(isfinite(d) && floor(d) == d &&
                    d >= -9223372036854775808.0 && d < 9223372036854775808.0))) {
                    R[instr_a(in)] = val_float(d);
                    VM_DISPATCH();
                }
                R[instr_a(in)] = val_float((double)(int64_t)d); /* int64 内整数结果 */
                VM_DISPATCH();
            }
            const Value r = op_pow_slow(ctx, fr, pc, vb, vc);
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(NEG) {
            const Value v = R[instr_b(in)];
            if (MICA_LIKELY(val_is_float(v))) { /* number 区（int53/float）直接取负 */
                R[instr_a(in)] = val_float(-val_as_float(v));
                VM_DISPATCH();
            }
            const Value r = value_neg_slow(ctx, fr, pc, v);
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(NOT) {
            const Value v = R[instr_b(in)];
            const uint32_t t = MICA_ITYPE(v);
            /* 严格布尔：仅 bool 合法，其它类型报错。is_bool = (t-TTRUE)<2（TTRUE/TFALSE 相邻）。 */
            if (MICA_UNLIKELY((uint32_t)(t - MICA_TTRUE) >= 2u)) {
                vm_truthy_type_error(ctx, fr, pc, v);
                R[instr_a(in)] = val_null();
                VM_DISPATCH();
            }
            R[instr_a(in)] = val_bool(t == MICA_TFALSE);
            VM_DISPATCH();
        }
        VM_CASE(ADDVN) {
            const Value vb = R[instr_b(in)], vc = ch->num_consts[instr_c(in)];
            if (MICA_LIKELY(val_both_num(vb, vc))) {
                R[instr_a(in)] = val_float(val_as_float(vb) + val_as_float(vc));
                VM_DISPATCH();
            }
            const Value r = op_add_slow(ctx, fr, pc, vb, vc);
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(SUBVN) {
            const Value vb = R[instr_b(in)], vc = ch->num_consts[instr_c(in)];
            if (MICA_LIKELY(val_both_num(vb, vc))) {
                R[instr_a(in)] = val_float(val_as_float(vb) - val_as_float(vc));
                VM_DISPATCH();
            }
            const Value r = op_sub_slow(ctx, fr, pc, vb, vc);
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(MULVN) {
            const Value vb = R[instr_b(in)], vc = ch->num_consts[instr_c(in)];
            if (MICA_LIKELY(val_both_num(vb, vc))) {
                R[instr_a(in)] = val_float(val_as_float(vb) * val_as_float(vc));
                VM_DISPATCH();
            }
            const Value r = op_mul_slow(ctx, fr, pc, vb, vc);
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(DIVVN) {
            const Value vb = R[instr_b(in)], vc = ch->num_consts[instr_c(in)];
            if (MICA_LIKELY(val_both_num(vb, vc))) {
                const double db = val_as_float(vc);
                if (MICA_LIKELY(db != 0)) {
                    R[instr_a(in)] = val_float(val_as_float(vb) / db);
                    VM_DISPATCH();
                }
            }
            const Value r = op_div_slow(ctx, fr, pc, vb, vc);
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(IDIVVN) {
            const Value vb = R[instr_b(in)], vc = ch->num_consts[instr_c(in)];
            if (MICA_LIKELY(val_both_num(vb, vc))) {
                const double db = val_as_float(vc);
                if (MICA_LIKELY(db != 0)) {
                    R[instr_a(in)] = val_float(floor(val_as_float(vb) / db));
                    VM_DISPATCH();
                }
            }
            const Value r = op_idiv_slow(ctx, fr, pc, vb, vc);
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(MODVN) {
            const Value vb = R[instr_b(in)], vc = ch->num_consts[instr_c(in)];
            if (MICA_LIKELY(val_both_num(vb, vc))) {
                const double db = val_as_float(vc);
                if (MICA_LIKELY(db != 0)) {
                    double rm = fmod(val_as_float(vb), db);
                    if (rm != 0 && ((rm < 0) != (db < 0))) rm += db;
                    R[instr_a(in)] = val_float(rm);
                    VM_DISPATCH();
                }
            }
            const Value r = op_mod_slow(ctx, fr, pc, vb, vc);
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(ADDNV) {
            const Value vb = ch->num_consts[instr_b(in)], vc = R[instr_c(in)];
            if (MICA_LIKELY(val_both_num(vb, vc))) {
                R[instr_a(in)] = val_float(val_as_float(vb) + val_as_float(vc));
                VM_DISPATCH();
            }
            const Value r = op_add_slow(ctx, fr, pc, vb, vc);
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(MULNV) {
            const Value vb = ch->num_consts[instr_b(in)], vc = R[instr_c(in)];
            if (MICA_LIKELY(val_both_num(vb, vc))) {
                R[instr_a(in)] = val_float(val_as_float(vb) * val_as_float(vc));
                VM_DISPATCH();
            }
            const Value r = op_mul_slow(ctx, fr, pc, vb, vc);
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(INC) {
            const Value v = R[instr_a(in)];
            if (MICA_LIKELY(val_is_float(v))) { /* number 区（int53/float）直接 +1.0 */
                R[instr_a(in)] = val_float(val_as_float(v) + 1.0);
                VM_DISPATCH();
            }
            const Value r = op_add_slow(ctx, fr, pc, v, val_int(1));
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(DEC) {
            const Value v = R[instr_a(in)];
            if (MICA_LIKELY(val_is_float(v))) {
                R[instr_a(in)] = val_float(val_as_float(v) - 1.0);
                VM_DISPATCH();
            }
            const Value r = op_sub_slow(ctx, fr, pc, v, val_int(1));
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(EQ) {
            const Value vb = R[instr_b(in)], vc = R[instr_c(in)];
            if (MICA_LIKELY(!val_is_instance(vb) && !val_is_instance(vc))) {
                R[instr_a(in)] = val_bool(value_eq(vb, vc));
                VM_DISPATCH();
            }
            const Value r = vm_cmp_op_slow(ctx, fr, pc, vb, vc, 0);
            VM_SYNC_FRAME(); /* 重载压帧可能 realloc 栈 */
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(NE) {
            const Value vb = R[instr_b(in)], vc = R[instr_c(in)];
            if (MICA_LIKELY(!val_is_instance(vb) && !val_is_instance(vc))) {
                R[instr_a(in)] = val_bool(!value_eq(vb, vc));
                VM_DISPATCH();
            }
            const Value r = vm_cmp_op_slow(ctx, fr, pc, vb, vc, 1);
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(LT) {
            const Value vb = R[instr_b(in)], vc = R[instr_c(in)];
            if (MICA_LIKELY(val_both_num(vb, vc))) {
                R[instr_a(in)] = val_bool(val_as_float(vb) < val_as_float(vc));
                VM_DISPATCH();
            }
            const Value r = vm_cmp_op_slow(ctx, fr, pc, vb, vc, 2); /* 字符串/实例重载/报错 */
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(LE) {
            const Value vb = R[instr_b(in)], vc = R[instr_c(in)];
            if (MICA_LIKELY(val_both_num(vb, vc))) {
                R[instr_a(in)] = val_bool(val_as_float(vb) <= val_as_float(vc));
                VM_DISPATCH();
            }
            const Value r = vm_cmp_op_slow(ctx, fr, pc, vb, vc, 3);
            VM_SYNC_FRAME();
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        /* GT/GE 已删（LuaJIT 反向操作数：codegen 交换操作数用 LT/LE） */
        VM_CASE(ISLT) {
            const Value a = R[instr_a(in)], b = R[instr_b(in)];
            if (MICA_LIKELY(val_both_num(a, b))) {
                if (val_as_float(a) < val_as_float(b)) pc++;
                VM_DISPATCH();
            }
            if (vm_cmp_true_slow(ctx, fr, pc, a, b, 2)) pc++;
            VM_DISPATCH();
        }
        VM_CASE(ISLE) {
            const Value a = R[instr_a(in)], b = R[instr_b(in)];
            if (MICA_LIKELY(val_both_num(a, b))) {
                if (val_as_float(a) <= val_as_float(b)) pc++;
                VM_DISPATCH();
            }
            if (vm_cmp_true_slow(ctx, fr, pc, a, b, 3)) pc++;
            VM_DISPATCH();
        }
        VM_CASE(ISEQ) {
            const Value a = R[instr_a(in)], b = R[instr_b(in)];
            if (MICA_LIKELY(!val_is_instance(a) && !val_is_instance(b))) {
                if (value_eq(a, b)) pc++;
            } else if (vm_cmp_true_slow(ctx, fr, pc, a, b, 0)) pc++;
            VM_DISPATCH();
        }
        VM_CASE(ISNE) {
            const Value a = R[instr_a(in)], b = R[instr_b(in)];
            if (MICA_LIKELY(!val_is_instance(a) && !val_is_instance(b))) {
                if (!value_eq(a, b)) pc++;
            } else if (vm_cmp_true_slow(ctx, fr, pc, a, b, 1)) pc++;
            VM_DISPATCH();
        }
        VM_CASE(ISLT_LEN) {
            int64_t len;
            const Value v = R[instr_b(in)];
            if (MICA_LIKELY(val_is_array(v))) len = (int64_t)((ObjArray *)val_as_obj(v))->count;
            else if (val_is_string(v)) len = (int64_t)((ObjString *)val_as_obj(v))->len;
            else {
                if (!vm_obj_length(ctx, fr, pc, v, &len))
                    VM_DISPATCH();
            }
            if (!val_is_num(R[instr_a(in)])) {
                runtime_error_at(ctx, fr, pc, "length comparison index must be an integer");
                VM_DISPATCH();
            }
            const int64_t x = (int64_t)val_as_float(R[instr_a(in)]);
            if (x < len) pc++;
            VM_DISPATCH();
        }
        VM_CASE(ISEQN) { /* R[a] == 数字常量（num 池）：单类型检查（AD，bx=num 下标） */
            const Value a = R[instr_a_ad(in)], k = ch->num_consts[instr_bx(in)];
            if (MICA_LIKELY(val_is_num(a))) {
                if (val_as_float(a) == val_as_float(k)) pc++;
            } else if (MICA_UNLIKELY(val_is_instance(a))) {
                if (vm_cmp_true_slow(ctx, fr, pc, a, k, 0)) pc++;
            }
            VM_DISPATCH();
        }
        VM_CASE(ISNEN) {
            const Value a = R[instr_a_ad(in)], k = ch->num_consts[instr_bx(in)];
            if (MICA_LIKELY(val_is_num(a))) {
                if (val_as_float(a) != val_as_float(k)) pc++;
            } else if (MICA_UNLIKELY(val_is_instance(a))) {
                if (vm_cmp_true_slow(ctx, fr, pc, a, k, 1)) pc++;
            }
            VM_DISPATCH();
        }
        VM_CASE(ISEQS) { /* R[a] == 字符串常量（str 池）：obj_string_eq（指针+hash+len） */
            const Value a = R[instr_a_ad(in)], k = ch->str_consts[instr_bx(in)];
            if (MICA_LIKELY(val_is_string(a))) {
                if (obj_string_eq((const ObjString *)val_as_obj(a), (const ObjString *)val_as_obj(k)))
                    pc++;
            } else if (MICA_UNLIKELY(val_is_instance(a))) {
                if (vm_cmp_true_slow(ctx, fr, pc, a, k, 0)) pc++;
            }
            VM_DISPATCH();
        }
        VM_CASE(ISNES) {
            const Value a = R[instr_a_ad(in)], k = ch->str_consts[instr_bx(in)];
            if (MICA_LIKELY(val_is_string(a))) {
                if (!obj_string_eq((const ObjString *)val_as_obj(a), (const ObjString *)val_as_obj(k)))
                    pc++;
            } else if (MICA_UNLIKELY(val_is_instance(a))) {
                if (vm_cmp_true_slow(ctx, fr, pc, a, k, 1)) pc++;
            }
            VM_DISPATCH();
        }

        VM_CASE(GET_MEMBER) {
            const Value obj = R[instr_c(in)];
            if (MICA_UNLIKELY(val_is_instance(obj))) {
                ObjClass* kl = ((ObjInstance *)val_as_obj(obj))->klass;
                /* 调用点 IC（D）：命中免 field_index 哈希 + 方法继承链。
                   IC 命中即确认无 getter（否则 miss 时 getter 会先接管，不会回填 IC）。 */
                CallSiteIC* ic = vm_ic_slot((Proto *)ch, pc);
                if (MICA_LIKELY(ic->klass == kl)) {
                    if (ic->kind == 1)
                        R[instr_a(in)] = ((ObjInstance *)val_as_obj(obj))->fields[ic->field_idx];
                    else
                        R[instr_a(in)] = obj_closure_val(ic->method);
                    VM_DISPATCH();
                }
                const String* name = &ch->ident_names[instr_b(in)];
                if (MICA_UNLIKELY(kl->has_getters)) {
                    if (vm_cold_get_member_getter(ctx, fr, pc, in, obj, name)) {
                        VM_SYNC_FRAME(); /* getter 压帧 + ensure_stack 可能 realloc 栈 */
                        VM_DISPATCH();
                    }
                }
                const int idx = obj_class_field_index(kl, name);
                if (idx >= 0) {
                    ic->klass = kl; ic->kind = 1; ic->field_idx = (int16_t)idx;
                    R[instr_a(in)] = ((ObjInstance *)val_as_obj(obj))->fields[idx];
                    VM_DISPATCH();
                }
                ObjClosure* m = obj_class_find_method_kv(kl, NAME_KV(name));
                if (m) {
                    ic->klass = kl; ic->kind = 2; ic->method = m;
                    R[instr_a(in)] = obj_closure_val(m);
                    VM_DISPATCH();
                }
                const char* cname = kl->name ? kl->name->chars : "?";
                runtime_error_at(ctx, fr, pc, "class %s has no member '%s'", cname, str_data(name));
                VM_DISPATCH();
            }
            /* 类静态计算属性 getter（let r => body；static） */
            if (MICA_UNLIKELY(val_is_obj(obj) && val_as_obj(obj)->type == OBJ_CLASS)) {
                ObjClass* kl = (ObjClass *)val_as_obj(obj);
                if (MICA_UNLIKELY(kl->has_getters)) {
                    const String* gname = &ch->ident_names[instr_b(in)];
                    if (vm_cold_get_member_class_getter(ctx, fr, pc, in, kl, gname)) {
                        VM_SYNC_FRAME(); /* getter 压帧 + ensure_stack 可能 realloc 栈 */
                        VM_DISPATCH();
                    }
                }
            }
            const String* name = &ch->ident_names[instr_b(in)];
            const Value r = get_member(ctx, fr, pc, obj, name);
            VM_SYNC_FRAME(); /* 反射 invoke（k_custom_member）可能压帧 + realloc 栈；先取结果再写 R */
            R[instr_a(in)] = r;
            VM_DISPATCH();
        }
        VM_CASE(SET_MEMBER) {
            const String* name = &ch->ident_names[instr_b(in)];
            const Value obj = R[instr_c(in)];
            const Value val = R[instr_a(in)];
            /* 调用点 IC（P1-②）：实例字段写命中直接 fields[idx]=v+写屏障（免 field_index
               单槽缓存）。IC 命中即确认无 setter/只读；miss 走冷 helper（i-cache 紧凑）。 */
            if (MICA_UNLIKELY(val_is_instance(obj))) {
                ObjClass* klass = ((ObjInstance *)val_as_obj(obj))->klass;
                CallSiteIC* ic = vm_ic_slot((Proto *)ch, pc);
                if (MICA_LIKELY(ic->klass == klass && ic->kind == 1)) {
                    ((ObjInstance *)val_as_obj(obj))->fields[ic->field_idx] = val;
                    if (val_is_obj(val)) gc_record_write(ctx->gc, (Obj *)val_as_obj(obj), val);
                    VM_DISPATCH();
                }
                vm_cold_set_member_miss(ctx, fr, pc, in, obj, val, name);
                VM_SYNC_FRAME(); /* miss 路径可能压 setter 帧 / realloc 栈 */
                VM_DISPATCH();
            }
            set_member(ctx, fr, pc, obj, name, val);
            VM_DISPATCH();
        }
        VM_CASE(LENGTH) {
            const Value v = R[instr_b(in)];
            if (MICA_LIKELY(val_is_array(v))) { /* 提升类型直接检查，省 val_is_obj + o->type */
                R[instr_a(in)] = val_int((int64_t)((ObjArray *)val_as_obj(v))->count);
                VM_DISPATCH();
            }
            if (MICA_LIKELY(val_is_string(v))) {
                R[instr_a(in)] = val_int((int64_t)((ObjString *)val_as_obj(v))->len);
                VM_DISPATCH();
            }
            int64_t len;
            if (!vm_obj_length(ctx, fr, pc, v, &len))
                VM_DISPATCH();
            R[instr_a(in)] = val_int(len);
            VM_DISPATCH();
        }
        VM_CASE(JMP) {
            const int32_t sbx = instr_sbx(in);
            pc += sbx;
            VM_DISPATCH();
        }
        VM_CASE(JMP_IF_FALSE) {
            const Value v = R[instr_a_ad(in)];
            const uint32_t t = MICA_ITYPE(v);
            /* 严格布尔：仅 bool 合法，其它类型报错。is_bool = (t-TTRUE)<2。 */
            if (MICA_UNLIKELY((uint32_t)(t - MICA_TTRUE) >= 2u)) {
                vm_truthy_type_error(ctx, fr, pc, v);
                VM_DISPATCH();
            }
            if (t == MICA_TFALSE) pc += instr_sbx(in);
            VM_DISPATCH();
        }
        VM_CASE(JMP_IF_TRUE) {
            const Value v = R[instr_a_ad(in)];
            const uint32_t t = MICA_ITYPE(v);
            if (MICA_UNLIKELY((uint32_t)(t - MICA_TTRUE) >= 2u)) {
                vm_truthy_type_error(ctx, fr, pc, v);
                VM_DISPATCH();
            }
            if (t == MICA_TTRUE) pc += instr_sbx(in);
            VM_DISPATCH();
        }
        VM_CASE(FORI) {
            /* 数值 for 初始化：step≠0 检查；start 超出范围（正步长 start>limit /
               负步长 start<limit）则循环不迭代（不跳 body）；否则 R[a+3]=R[a+2]，跳 body。
               P2：step/start/limit 全 number 时直接 double 比较（免 compare_values 三态 + 错误分支）。 */
            const int a = instr_a_ad(in);
            const Value vstep = R[a], vstart = R[a + 2], vlimit = R[a + 1];
            if (MICA_LIKELY(val_both_num(vstep, vstart) && val_is_num(vlimit))) {
                const double step = val_as_float(vstep);
                if (MICA_UNLIKELY(step == 0.0)) {
                    runtime_error_at(ctx, fr, pc, "step cannot be zero");
                    VM_DISPATCH();
                }
                const double start = val_as_float(vstart), limit = val_as_float(vlimit);
                if ((step > 0 && start > limit) || (step < 0 && start < limit))
                    VM_DISPATCH(); /* 不进入循环 */
                R[a + 3] = R[a + 2];
                pc += instr_sbx(in);
                VM_DISPATCH();
            }
            /* 通用路径：字符串范围/重载等（compare_values 全类型） */
            {
                const int sc = compare_values(ctx, fr, pc, vstep, val_int(0));
                if (CTX_HAS_ERROR(ctx))
                    VM_DISPATCH();
                if (sc == 0) {
                    runtime_error_at(ctx, fr, pc, "step cannot be zero");
                    VM_DISPATCH();
                }
                const int cc = compare_values(ctx, fr, pc, vstart, vlimit);
                if (CTX_HAS_ERROR(ctx))
                    VM_DISPATCH();
                if ((sc > 0 && cc > 0) || (sc < 0 && cc < 0))
                    VM_DISPATCH();
                R[a + 3] = R[a + 2];
                pc += instr_sbx(in);
                VM_DISPATCH();
            }
        }
        VM_CASE(FORL) {
            /* 数值 for 循环控制：R[a+2]+=R[a]；按步长符号比较 R[a+2] 与 R[a+1]，
               满足则 R[a+3]=R[a+2] 并跳循环体，否则退出。
               P1-③：只检查 step+counter 是 number（2 次 itype），limit 信任 FORI 已检查
               （Lua FORLOOP 同款——循环体改 limit 是未定义行为；counter/step 被改坏仍走通用路径报错）。 */
            const int a = instr_a_ad(in);
            const Value vs = R[a], vc2 = R[a + 2];
            if (MICA_LIKELY(val_both_num(vs, vc2))) {
                const double step = val_as_float(vs);
                const double c = val_as_float(vc2) + step;
                R[a + 2] = val_float(c);
                const double vl = val_as_float(R[a + 1]);
                if ((step > 0 && c <= vl) || (step < 0 && c >= vl)) {
                    R[a + 3] = R[a + 2];
                    pc += instr_sbx(in);
                }
                VM_DISPATCH();
            }
            /* 通用路径：字符串/重载等 */
            {
                const Value vl = R[a + 1];
                if (MICA_LIKELY(val_both_num(vs, vc2))) {
                    R[a + 2] = val_float(val_as_float(vc2) + val_as_float(vs));
                } else {
                    R[a + 2] = op_add(ctx, fr, pc, vc2, vs);
                    if (CTX_HAS_ERROR(ctx))
                        VM_DISPATCH();
                    VM_SYNC_FRAME(); /* op_add 慢路径（重载）可能 realloc 栈 */
                }
                const int cc = compare_values(ctx, fr, pc, R[a + 2], vl);
                if (CTX_HAS_ERROR(ctx))
                    VM_DISPATCH();
                const int sc = compare_values(ctx, fr, pc, vs, val_int(0));
                if (CTX_HAS_ERROR(ctx))
                    VM_DISPATCH();
                if ((sc > 0 && cc <= 0) || (sc < 0 && cc >= 0)) {
                    R[a + 3] = R[a + 2];
                    pc += instr_sbx(in);
                }
                VM_DISPATCH();
            }
        }
        VM_CASE(CALL) {
            /* 调用分派统一走 vm_call_dispatch（receiver 显式在 slot1）：
               普通函数压帧、native/builtin/协程方法直接执行、类构造建实例。
               check_throws：OP_CALL 未受保护时 true（throws 检查在 dispatch 的 closure 分支），
               受保护时豁免；OP_FASTCALL 编译期已排除 throws，恒 false。
               pc 传给 dispatch：顶部 fr->savedpc = pc 持久化 caller 恢复点（压 callee 帧前）。 */
            vm_call_dispatch(ctx, fr, pc, R[instr_a(in)], instr_a(in), instr_b(in),
                             instr_c(in) & RESULT_MASK, !(instr_c(in) & RESULT_PROTECT));
            if (MICA_UNLIKELY(ctx->yield_requested)) {
                /* Phase 2c：native 调 mica_yield/mica_callk 挂起 → 产出值 + 返回 CORO_YIELDED */
                ctx->yield_requested = false;
                ctx->yield_res_reg = instr_a(in); /* 续体结果写入槽 */
                if (out) *out = ctx->yield_value;
                ctx->active_jmp = prev_jmp;
                return CORO_YIELDED;
            }
            VM_SYNC_FRAME(); /* vm_call_dispatch 压帧 / native 可能 realloc 栈 */
            VM_DISPATCH();
        }
        VM_CASE(FASTCALL) {
            vm_call_dispatch(ctx, fr, pc, R[instr_a(in)], instr_a(in), instr_b(in),
                             instr_c(in) & RESULT_MASK, true);
            if (MICA_UNLIKELY(ctx->yield_requested)) {
                ctx->yield_requested = false;
                ctx->yield_res_reg = instr_a(in);
                if (out) *out = ctx->yield_value;
                ctx->active_jmp = prev_jmp;
                return CORO_YIELDED;
            }
            VM_SYNC_FRAME(); /* vm_call_dispatch 压帧 / native 可能 realloc 栈 */
            VM_DISPATCH();
        }
        VM_CASE(RET0) { /* 返回 0 个值（c=RESULT_DISCARD，a 未用）：特化免 VARARGS 三元+copy=min */
            CallFrame* cf = ctx_frame(ctx);
            Value* base = cf->base;
            if (MICA_UNLIKELY((cf->flags & CF_DEFER) ||
                              (!(cf->flags & CF_GETTER) && (cf->u.ctor.constructed || cf->u.ctor.after)))) {
                if (vm_cold_return_special(ctx, cf, pc, base, stop_count)) { done = true; goto vm_done; }
                VM_SYNC_FRAME();
                VM_DISPATCH();
            }
            /* actual 恒 0：count 由调用方 want 决定（VARARGS→0、DISCARD→0、n→n） */
            const int count = (cf->want == RESULT_VARARGS || cf->want == RESULT_DISCARD) ? 0 : cf->want;
            if (MICA_UNLIKELY(ctx->opencapval != NULL)) obj_frame_close_capvals(ctx->gc, ctx, cf);
            for (int i = 0; i < count; i++) base[i] = val_null();
            ctx->stack_top = base + count;
            ctx_frame_pop(ctx);
            if (MICA_LIKELY(!(cf->flags & CF_GETTER) && ctx->defer_top <= cf->defer_mark)) {
                for (int i = count; i < (int)cf->closure->proto->max_regs; i++)
                    base[i] = val_null();
            } else {
                if (vm_cold_return_finish(ctx, cf, pc, base, base, count, cf->flags & CF_GETTER)) goto vm_done;
            }
            if (ctx->frame_count <= stop_count) { done = true; goto vm_done; }
            VM_SYNC_FRAME();
            VM_DISPATCH();
        }
        VM_CASE(RET1) { /* 返回 1 个值（c=1，a=结果寄存器）：特化免 VARARGS 三元+memmove */
            CallFrame* cf = ctx_frame(ctx);
            Value* base = cf->base;
            if (MICA_UNLIKELY((cf->flags & CF_DEFER) ||
                              (!(cf->flags & CF_GETTER) && (cf->u.ctor.constructed || cf->u.ctor.after)))) {
                if (vm_cold_return_special(ctx, cf, pc, base, stop_count)) { done = true; goto vm_done; }
                VM_SYNC_FRAME();
                VM_DISPATCH();
            }
            int count;
            if (cf->want == RESULT_VARARGS) count = 1;
            else if (cf->want == RESULT_DISCARD) count = 0;
            else count = cf->want;
            Value* src = base + instr_a(in);
            if (MICA_UNLIKELY(ctx->opencapval != NULL)) obj_frame_close_capvals(ctx->gc, ctx, cf);
            vm_check_uncaught_error(ctx, cf, pc, *src);
            if (count > 0) *base = *src;
            for (int i = 1; i < count; i++) base[i] = val_null();
            ctx->stack_top = base + count;
            ctx_frame_pop(ctx);
            if (MICA_LIKELY(!(cf->flags & CF_GETTER) && ctx->defer_top <= cf->defer_mark)) {
                for (int i = count; i < (int)cf->closure->proto->max_regs; i++)
                    base[i] = val_null();
            } else {
                if (vm_cold_return_finish(ctx, cf, pc, base, src, count, cf->flags & CF_GETTER)) goto vm_done;
            }
            if (ctx->frame_count <= stop_count) { done = true; goto vm_done; }
            VM_SYNC_FRAME();
            VM_DISPATCH();
        }
        VM_CASE(RETURN) {
            CallFrame* cf = ctx_frame(ctx);
            Value* base = cf->base;
            /* defer / 构造器帧返回：特殊帧处理（冷路径外移） */
            if (MICA_UNLIKELY((cf->flags & CF_DEFER) ||
                              (!(cf->flags & CF_GETTER) &&
                               (cf->u.ctor.constructed || cf->u.ctor.after)))) {
                if (vm_cold_return_special(ctx, cf, pc, base, stop_count)) {
                    done = true;
                    goto vm_done;
                }
                VM_SYNC_FRAME(); /* 特殊帧弹出 / defaults→init 压帧 */
                VM_DISPATCH();
            }
            /* 普通返回热路径（多值/vararg） */
            Value* src = base + instr_a(in);
            const int actual =
                    (instr_c(in) == RESULT_VARARGS) ? (int)(ctx->stack_top - src) : (int)instr_c(in);
            int count;
            if (cf->want == RESULT_VARARGS) count = actual;
            else if (cf->want == RESULT_DISCARD) count = 0;
            else count = cf->want;
            const int copy = actual < count ? actual : count;
            /* 先关闭开放捕获值（见上） */
            if (MICA_UNLIKELY(ctx->opencapval != NULL))
                obj_frame_close_capvals(ctx->gc, ctx, cf);
            vm_check_uncaught_error(ctx, cf, pc, *src);
            if (copy > 0) memmove(&*base, src, (size_t)copy * sizeof(Value));
            for (int i = copy; i < count; i++) base[i] = val_null();
            ctx->stack_top = base + count;
            ctx_frame_pop(ctx);
            if (MICA_LIKELY(!(cf->flags & CF_GETTER) && ctx->defer_top <= cf->defer_mark)) {
                for (int i = count; i < (int)cf->closure->proto->max_regs; i++)
                    base[i] = val_null();
            } else {
                if (vm_cold_return_finish(ctx, cf, pc, base, src, count, cf->flags & CF_GETTER)) {
                    goto vm_done;
                }
            }
            if (ctx->frame_count <= stop_count) {
                done = true;
                goto vm_done;
            }
            VM_SYNC_FRAME();
            VM_DISPATCH();
        }
        VM_CASE(CLOSURE) {
            vm_cold_closure(ctx, fr, in);
            VM_DISPATCH();
        }
        VM_CASE(GET_CAPVAL) {
            vm_cold_get_capval(ctx, fr, in);
            VM_DISPATCH();
        }
        VM_CASE(SET_CAPVAL) {
            vm_cold_set_capval(ctx, fr, in);
            VM_DISPATCH();
        }
        VM_CASE(VARARGS) {
            vm_cold_varargs(ctx, fr, in);
            VM_DISPATCH();
        }
        VM_CASE(CONCAT) {
            R[instr_a(in)] = concat_n(ctx, fr, pc, &R[instr_b(in)], instr_c(in) + 1);
            VM_DISPATCH();
        }
        VM_CASE(GET_METHOD) {
            vm_cold_get_method(ctx, fr, pc, in);
            VM_DISPATCH();
        }
        VM_CASE(ARRAY) {
            vm_cold_array(ctx, fr, in);
            VM_DISPATCH();
        }
        VM_CASE(MAP) {
            vm_cold_map(ctx, fr, in);
            VM_DISPATCH();
        }
        VM_CASE(ERROR) {
            vm_cold_error(ctx, fr, in);
            VM_DISPATCH();
        }
        VM_CASE(NEW_ERROR) {
            vm_cold_new_error(ctx, fr, in);
            VM_DISPATCH();
        }
        VM_CASE(NEW_ERROR_NAMED) {
            VM_CALL_COLD(vm_cold_new_error_named(ctx, fr, in)); /* 消费 savedpc data-word + 可能报错 */
            VM_DISPATCH();
        }
        VM_CASE(ERROR_INFO) {
            vm_cold_error_info(ctx, fr, pc, in);
            VM_DISPATCH();
        }
        VM_CASE(IS_ERROR) {
            vm_cold_is_error(ctx, fr, in);
            VM_DISPATCH();
        }
        VM_CASE(THROW) {
            vm_cold_throw(ctx, fr, pc, in);
            VM_DISPATCH();
        }
        VM_CASE(MATCH_ERROR) {
            VM_CALL_COLD(vm_cold_match_error(ctx, fr, in)); /* 消费 savedpc data-word + 可能报错 */
            VM_DISPATCH();
        }
        VM_CASE(GET_ERROR_PAYLOAD) {
            VM_CALL_COLD(vm_cold_get_error_payload(ctx, fr, in)); /* 消费 savedpc data-word + 可能报错 */
            VM_DISPATCH();
        }
        VM_CASE(NEW_ENUM_NAMED) {
            VM_CALL_COLD(vm_cold_new_enum_named(ctx, fr, in)); /* 消费 savedpc data-word + 可能报错 */
            VM_DISPATCH();
        }
        VM_CASE(ENUM_INFO) {
            vm_cold_enum_info(ctx, fr, pc, in);
            VM_DISPATCH();
        }
        VM_CASE(GET_ENUM_PAYLOAD) {
            vm_cold_get_enum_payload(ctx, fr, in);
            VM_DISPATCH();
        }
        VM_CASE(MATCH_ENUM) {
            VM_CALL_COLD(vm_cold_match_enum(ctx, fr, in)); /* 消费 savedpc data-word */
            VM_DISPATCH();
        }
        VM_CASE(MATCH_LITERAL) {
            vm_cold_match_literal(ctx, fr, in);
            VM_DISPATCH();
        }
        VM_CASE(IS_TYPE) {
            vm_cold_is_type(ctx, fr, in);
            VM_DISPATCH();
        }
        VM_CASE(YIELD) {
            if (!CTX_IS_CORO(ctx)) {
                runtime_error_at(ctx, fr, pc, "yield outside coroutine");
                VM_DISPATCH();
            }
            fr->savedpc = pc; /* 挂起：持久化恢复点（resume 读 yf->savedpc[-1] 回写参数） */
            if (out) *out = R[instr_b(in)];
            ctx->active_jmp = prev_jmp; /* yield 是绕过 vm_done 的正常退出：恢复外层跳转缓冲 */
            return CORO_YIELDED;
        }
        VM_CASE(DEFER_MARK) {
            vm_cold_defer_mark(ctx, fr, in);
            VM_DISPATCH();
        }
        VM_CASE(DEFER_PUSH) {
            vm_cold_defer_push(ctx, fr, in);
            VM_DISPATCH();
        }
        VM_CASE(DEFER_RUN) {
            vm_cold_defer_run(ctx, fr, pc, in);
            if (CTX_HAS_ERROR(ctx)) vm_rethrow_error(ctx); /* defer 闭包错误：嵌套 vm_run 捕获后重抛 */
            VM_SYNC_FRAME(); /* defer 闭包嵌套 vm_run 可能 realloc 栈 */
            VM_DISPATCH();
        }
        VM_CASE(EXIT) {
            /* 入口程序结束：清入口帧（关闭捕获值、清寄存器、运行 defer）、弹出（冷路径外移） */
            vm_cold_exit(ctx, fr, pc);
            done = true;
            goto vm_done;
        }
        VM_CASE(TAILCALL) {
            /* 尾调用：return f(...)。codegen 在"被调函数是静态已知的顶层 func 且不在
               try/catch 内"时发射本指令，并在其后紧跟 OP_RETURN 传播结果。
               ⚠️ 帧复用（vm_cold_tailcall_reuse）**已停用**：它的实参搬运与
               vm_call_dispatch 的槽布局约定不一致——codegen 把 self（普通函数为 null）
               放在 slot1、用户实参从 slot2 起，argc 是**含 self 槽**的总数；帧复用按
               `memmove(base[1], base[a+1], argc)` 搬运会把 self 空洞一并搬入，实测
               `func tick(p){return p>0}` + `return tick(prog)` 的 p 取到 null
               （"cannot compare number with null"）。在搬运/布局彻底对齐前统一走普通
               调用：语义正确，代价是尾递归不再是 O(1) 栈（Mica 的帧块是堆分配的，
               深递归仍可用，只是受 MICA_FRAME_MAX_SOFT 限制）。
               下方 OP_RETURN 完成结果传播（defers 由它跑，避免双跑）。 */
            const int a = instr_a(in);
            const int argc = instr_b(in);
            const Value fn = R[a];
            vm_call_dispatch(ctx, fr, pc, fn, a, argc, RESULT_VARARGS,
                             !(instr_c(in) & RESULT_PROTECT));
            if (MICA_UNLIKELY(ctx->yield_requested)) {
                ctx->yield_requested = false;
                ctx->yield_res_reg = a;
                if (out) *out = ctx->yield_value;
                ctx->active_jmp = prev_jmp;
                return CORO_YIELDED;
            }
            VM_SYNC_FRAME();
            VM_DISPATCH();
        }

#if VM_COMPUTED_GOTO
        L_VM_DEFAULT: /* 分发表兜底：未定义 opcode（OP_COUNT..255） */
            runtime_error_at(ctx, fr, pc, "unknown opcode %d", instr_op(in));
            goto vm_done; /* runtime_error 通常已 longjmp（active_jmp 有效）；此处兜底终止 */
#else
        default:
            runtime_error_at(ctx, fr, pc, "unknown opcode %d", instr_op(in));
            break;
        }
    }
#endif
vm_done:
    ctx->active_jmp = prev_jmp; /* 恢复外层活跃跳转缓冲（所有 goto vm_done 的正常退出路径） */
    if (CTX_HAS_ERROR(ctx)) {
        return CORO_ERROR;
    }
    if (out) *out = (ctx->frame_count == 0 || done) ? ctx->stack[0] : val_null();
    return done ? CORO_DONE : CORO_YIELDED;
}
