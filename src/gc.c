#include "gc.h"

#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_MSC_VER)
#include <windows.h>
#else
#include <time.h>
#endif

#include "state.h"   /* mica_context.ctx_next（contexts 链） */
#include "table.h"

#if MICA_GC_SIMD && (defined(__SSE2__) || defined(_M_X64))
#include <emmintrin.h> /* D7：SSE2 批量数组标记 */
#endif

/* 性能统计计时（gc_set_stats_enabled 启用；仅诊断，默认关闭） */
static double mica_now_us(void) {
#if defined(_MSC_VER)
    LARGE_INTEGER f, c;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return (double)c.QuadPart * 1e6 / (double)f.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e6 + (double)ts.tv_nsec / 1e3;
#endif
}

/* ============================================================
   真分代增量 GC（Lua 5.5 双白翻转 + 2 代边界指针）：
   - 双白翻转：mark 位 bit0/bit1 白（currentwhite 二选一）、bit2 黑、灰=0。
     每轮收集"标记完成、清扫开始"时翻转 currentwhite；清扫把存活对象重置为当前白、
     死对象保持旧白 → free。**无需 reset_all_marks，颜色自愈**。
   - 7 态渐进代龄（ObjHeader.age）：NEW→SURV→OLD1→OLD（OLD0 由写屏障/传播设置）。
     两代边界指针（all_obj 链上新→旧）：[all_obj,survival)=NEW、[survival,old1)=SURV、
     [old1,reallyold)=OLD0/OLD1（近期晋升，markold 有界重扫）、[reallyold,)=OLD。
   - 轮次驱动：young 每轮收集（真分代）、old 每 GC_OLD_INTERVAL 轮启动增量 major
     （三色分片标记 + 增量清扫 + 增量 rebuild）。**无独立中年代/无 mid 收集**。
   - 真分代 minor（O(根 + markold + young)，不碰 old）：
     写屏障把 old 写入的 young child 提升到 OLD0 并入 gray（当轮传播）；
     markold 有界重扫 [firstold1,reallyold) 的 OLD1 对象 force_trace（其 young 子对象 promote
     传播提升到 OLD0），然后推进 OLD——OLD1 自限，只存在一轮。
   - contexts 链：old 协程 context 的栈值可能被 resume 无屏障写入 young，每轮重扫
     （Lua 的 old 线程进 grayagain）；young context 由根传播经 contexts 链覆盖。
   - 根 = permanent 对象链（globals/module 表/主 context 都是 permanent）。
   ============================================================ */

/* 颜色谓词（双白 + 三色；见 gc.h 的 GC_WHITEBITS/GC_BLACK_BIT） */
#define iswhite(g, o) (((o)->mark & GC_WHITEBITS) == (g)->currentwhite)
#define isblack(o)    (((o)->mark & GC_BLACK_BIT) != 0)
#define isgray(o)     (((o)->mark & GC_MASKCOLORS) == 0)
#define otherwhite(g) ((g)->currentwhite ^ GC_WHITEBITS)
/* 重置为当前白（清 GC 颜色位 + 设当前白） */
#define resetwhite(g, o) ((o)->mark = (uint8_t)(((o)->mark & ~GC_MASKCOLORS) | (g)->currentwhite))

/* ---------- 内部工具 ---------- */

static size_t align16(size_t n) {
    return (n + 15) & ~(size_t)15;
}

/* 记录一次收集暂停（stats_enabled 时；环形缓冲满则停止，不覆盖旧样本） */
static void record_pause(Gc *gc, double us) {
    if (gc->pause_samples && gc->pause_n < GC_PAUSE_SAMPLES_MAX) {
        gc->pause_samples[gc->pause_n++] = us;
    }
}

/* 对象近似字节大小（含容器动态部分；Lua objsize 的 Mica 版，用于自适应 major 记账）。
   固定小对象统一按 align16(ObjHeader) 估算——记账只用于触发阈值，近似足够。 */
static size_t obj_size(const Obj *o) {
    switch (o->type) {
    case OBJ_STRING: {
        const ObjString *s = (const ObjString *)o;
        return align16(sizeof(ObjString) + s->len + 1);
    }
    case OBJ_ARRAY: {
        const ObjArray *a = (const ObjArray *)o;
        return align16(sizeof(ObjArray)) + a->capacity * sizeof(Value);
    }
    case OBJ_MAP: {
        const ObjMap *m = (const ObjMap *)o;
        return align16(sizeof(ObjMap)) + m->sparse_cap * sizeof(size_t) +
               m->cap * sizeof(ObjMapEntry);
    }
    case OBJ_NATIVE_INSTANCE: {
        const ObjNativeInstance *i = (const ObjNativeInstance *)o;
        return align16(sizeof(ObjNativeInstance) + i->klass->userdata_size);
    }
    default:
        return align16(sizeof(Obj)); /* 固定小对象近似 */
    }
}

void gc_step(Gc *gc);
static bool obj_is_grayable(const Gc *gc, const Obj *o);
static void propagate_gray(Gc *gc, GcTraceCtx *ctx);
static int propagate_gray_slice(Gc *gc, GcTraceCtx *ctx, int budget); /* OPT-1：分片 */
static void force_trace(Gc *gc, Obj *o, GcTraceCtx *ctx);

/* ---------- 分配 ---------- */

void *gc_alloc(Gc *gc, size_t size, ObjType type) {
    assert(type < GC_VTABLE_SLOTS);
    /* 分配前推进 GC 状态机（GC_ULTRA_PLAN D1 债务触发）。unsafe 构造窗口内不收集。
       enabled 恒 true、unsafe_depth 常 0、debt 常正 → MICA_UNLIKELY 把触发分支放冷区。
       热检查收敛为单条件（debt<=0），替代原 4 条件（phase/young_limit/字节阈值）。 */
    if (MICA_UNLIKELY(gc->enabled && gc->unsafe_depth == 0 && gc->debt <= 0)) {
        gc_step(gc);
    }
    size = align16(size);
    gc->alloc_bytes += size; /* 字节记账（统计 + 债务） */
    gc->debt -= (ptrdiff_t)size; /* D1：分配欠债（Lua lmem.c:187 `GCdebt -= nsize` 同款） */
    Obj *obj = malloc(size);
    if (MICA_UNLIKELY(obj == NULL)) {
        /* D5：紧急收集 + 重试（Lua tryagain，lmem.c:162-170）。
           收集嵌套（collect_depth>0）或分片执行中（step_depth>0）→ 跳过收集直报 OOM。 */
        if (gc->enabled && gc->collect_depth == 0 && gc->step_depth == 0) {
            gc_collect_major(gc); /* 全量紧急收集（尽力释放内存） */
            obj = malloc(size);
        }
        if (obj == NULL) {
            if (gc->oom_handler) gc->oom_handler(gc, size); /* 宿主可转 Mica 错误 */
            assert(!"out of memory"); /* 无处理器时的最后防线（Release 下宿主应注册处理器） */
            return NULL;
        }
    }
    /* 不 memset：工厂完全初始化业务字段。这里只设 GC 自有字段。
       gclist 按 phase 显式管理——MARKING 链接 gray、其余置 NULL（grayable 才有该字段）。 */
    obj->type = type;
    obj->age = MICA_AGE_NEW;
    obj->flags = 0;
    obj->next = gc->all_obj;
    gc->all_obj = obj;
    gc->stats.young_live_count++;
    /* GCv2 OPT-2：精确字节记账。live_bytes += align16(size) 实际占用（含容器动态部分
       由 gc_account_alloc 后续补充；align16 一致确保 alloc/free 对称）。 */
    gc->live_bytes += (int64_t)size;
    if (gc->phase == GC_MARKING) {
        obj->mark = 0;              /* 灰：向前屏障，入 gray 待标记 */
        if (obj_is_grayable(gc, obj)) {
            OBJ_GCLIST(obj) = gc->gray;
            gc->gray = obj;
        } else {
            obj->mark = GC_BLACK_BIT;
        }
    } else if (gc->phase == GC_SWEEPING) {
        obj->mark = GC_BLACK_BIT;   /* 向后屏障：清扫期新对象存活 */
        if (obj_is_grayable(gc, obj)) OBJ_GCLIST(obj) = NULL;
    } else {
        obj->mark = (uint8_t)gc->currentwhite;
        if (obj_is_grayable(gc, obj)) OBJ_GCLIST(obj) = NULL;
    }
    return obj;
}

void *gc_alloc_permanent(Gc *gc, size_t size, ObjType type) {
    assert(type < GC_VTABLE_SLOTS);
    size = align16(size);
    Obj *obj = malloc(size);
    assert(obj != NULL);
    obj->type = type;
    obj->age = MICA_AGE_OLD; /* permanent 的 age 不参与各代判断（permanent 链独立、永不回收） */
    obj->flags = OBJ_PERMANENT;
    obj->mark = 0;
    /* 永久字符串是叶子（无子对象），不进 permanent 根链（GC 根扫描不遍历它），
       单独挂 perm_strs 链，gc_deinit 统一释放。其余永久对象进 permanent 链作根。 */
    if (type == OBJ_STRING) {
        obj->next = gc->perm_strs;
        gc->perm_strs = obj;
    } else {
        obj->next = gc->permanent;
        gc->permanent = obj;
        if (obj_is_grayable(gc, obj)) OBJ_GCLIST(obj) = NULL;
    }
    return obj;
}

void gc_account_alloc(Gc *gc, size_t delta) {
    if (!gc->enabled || delta == 0) return;
    gc->alloc_bytes += delta;
    gc->debt -= (ptrdiff_t)delta; /* D3：容器增长也记账（Lua 每次 realloc 都记账，lmem.c:187） */
    /* D3：容器增长也驱动 GC——增量收集期且欠债时推进（修复"realloc 不推进 GC"的洞）。
       step_depth 防护：finalizer 内 realloc 容器不会重入 gc_step。 */
    if (MICA_UNLIKELY(gc->phase != GC_IDLE && gc->debt <= 0 && gc->step_depth == 0)) {
        gc_step(gc);
    }
}

/* ---------- GC_ULTRA_PLAN D2：显式根集 ---------- */

void gc_add_root(Gc *gc, Obj *o) {
    if (!o || !(o->flags & OBJ_PERMANENT)) return;
    for (int i = 0; i < gc->n_roots; i++) {
        if (gc->roots[i] == o) return; /* 幂等 */
    }
    /* 根集满：Release 下 assert 被消掉会**静默丢弃**该根 → 其引用的运行期对象被误回收。
       改为硬失败（宿主 API 误用时应当立即暴露，而非产生难以定位的悬挂引用）。 */
    if (gc->n_roots >= GC_ROOTS_MAX) {
        fprintf(stderr,
                "mica: GC root set overflow (GC_ROOTS_MAX=%d); root %p(type=%d) not registered\n",
                GC_ROOTS_MAX, (void *)o, o->type);
        abort();
    }
    gc->roots[gc->n_roots++] = o;
}

bool gc_is_root(const Gc *gc, const Obj *o) {
    for (int i = 0; i < gc->n_roots; i++) {
        if (gc->roots[i] == o) return true;
    }
    return false;
}

void gc_set_explicit_roots(Gc *gc, bool enabled) {
    gc->explicit_roots = enabled;
}

void gc_set_oom_handler(Gc *gc, void (*handler)(Gc *gc, size_t size)) {
    gc->oom_handler = handler;
}

#ifdef MICA_GC_DEBUG
/* 调试校验开关：抑制根覆盖校验（负例测试用；字段随 Gc calloc 零初始化 = 默认开）。 */
void gc_set_debug_checks(Gc *gc, bool enabled) {
    gc->debug_checks_enabled = enabled;
}
#endif

/* ---------- 三色标记 ---------- */

/* 有 gclist 的引用型对象可入 gray 链（统一偏移）。叶子/无引用类型不入 gray；
   小而原子可 trace 的类型（如 capval 只追一个值）也不入 gray，atomic trace 即够。 */
/* 内置 grayable 类型位掩码（有 gclist 可入 gray 链）：type < 32 用位查表（热路径，
   替代 15 类型 switch）。非 grayable：string/native/coro_method/builtin_method/weakref/capval。
   上层模块的自定义类型（如反射 LREFL_*）由各自 init 经 gc_register_obj_type 追加
   —— GC 核心不硬编码上层类型常量。 */
#define MICA_BUILTIN_GRAYABLE_BITS \
    ((1u << OBJ_ARRAY) | (1u << OBJ_MAP) | (1u << OBJ_TABLE) | (1u << OBJ_CLOSURE) | \
     (1u << OBJ_NATIVE_CLOSURE) | (1u << OBJ_ERROR) | (1u << OBJ_ENUM_VAL) | \
     (1u << OBJ_INSTANCE) | (1u << OBJ_CONTEXT))

static bool obj_is_grayable(const Gc *gc, const Obj *o) {
    if (o->type < 32) return (gc->grayable_bits >> o->type) & 1u;
    return false; /* 超出 32 位域的自定义类型默认非 grayable */
}

void gc_register_obj_type(Gc *gc, uint8_t type, bool grayable) {
    if (type >= 32) return; /* 位域上限（扩展需同步改 obj_is_grayable） */
    if (grayable) gc->grayable_bits |= (1u << type);
    else          gc->grayable_bits &= ~(1u << type);
}

static void trace_object(Gc *gc, Obj *o, GcTraceCtx *ctx) {
    if (o->type < GC_VTABLE_SLOTS && gc->vtables[o->type].trace) {
        gc->vtables[o->type].trace(o, ctx);
    }
}

/* 白→灰：灰对象入 gray 链（有 gclist）；无 gclist 但有 trace（自定义类型）原子 trace；
   叶子直接置黑。双白下"白"= 当前白；旧白对象（上一轮死色）iswhite false 不标记，清扫时回收。 */
static void major_push(Gc *gc, Obj *o) {
    if (o->flags & OBJ_PERMANENT) return;
    if (!iswhite(gc, o)) return;
    o->mark = 0; /* 灰 */
    if (obj_is_grayable(gc, o)) {
        OBJ_GCLIST(o) = gc->gray;
        gc->gray = o;
    } else if (o->type < GC_VTABLE_SLOTS && gc->vtables[o->type].trace) {
        GcTraceCtx ctx = { .gc = gc, .promote = false };
        trace_object(gc, o, &ctx);
        o->mark = GC_BLACK_BIT;
    } else {
        o->mark = GC_BLACK_BIT;
    }
}

/* 根对象（permanent 链）的分片播种见 gc_root_trace_slice；permanent 根只 trace 子对象
   （permanent 根如全局表/主 context 也须标记其运行期子对象；permanent 对象自身不标记）。 */

/* ---------- markold：重扫代际脏集合（真分代 minor 的核心） ---------- */

/* promote：把 young 子对象提升到 OLD0（age 0/1→2）。提升后它由 promote 传播置灰入 gray，
   递归提升其 own young 后代——跨代引用的 young 后代闭包整体提升到"渐进阶"，此后父代不再
   直接持有 young。OLD0 下轮 sweep 推进 OLD1（markold 重扫）再 OLD。 */
static void promote_young(Gc *gc, Obj *o) {
    if (o->flags & OBJ_PERMANENT) return;
    if (!AGE_IS_YOUNG(o->age)) return;
    o->age = MICA_AGE_OLD0;
    gc->stats.young_live_count--;
    gc->stats.old_live_count++;
}

/* 强制重扫（markold）：把对象置灰入 gray（无论当前黑白——black 的提升对象也须重扫其
   young 子对象）。已在 gray 链上的跳过（会被传播）。非 grayable 原子 trace 后置黑。 */
static void force_trace(Gc *gc, Obj *o, GcTraceCtx *ctx) {
    if (o->flags & OBJ_PERMANENT) return;
    if (isgray(o)) return; /* 已在 gray 链，会被传播 */
    if (obj_is_grayable(gc, o)) {
        o->mark = 0;
        OBJ_GCLIST(o) = gc->gray;
        gc->gray = o;
    } else if (o->type < GC_VTABLE_SLOTS && gc->vtables[o->type].trace) {
        trace_object(gc, o, ctx);
        o->mark = GC_BLACK_BIT;
    } else {
        o->mark = GC_BLACK_BIT;
    }
}

/* 排空 gray 链：trace 每个灰对象并置黑。ctx 决定 promote 模式（markold 传播提升 young 后代）。
   无环保障：每个灰对象弹出后置黑（mark=0 弹出，iswhite 失败不可重入），链严格收缩终止；
   旧版 200000 硬上限会静默留下灰对象 → 被清扫误回收（大堆 UAF），已删。
   注：此版本为 minor 原子收集专用 wrapper（传 INT_MAX）。增量分片见下面的 propagate_gray_slice。 */
static void propagate_gray(Gc *gc, GcTraceCtx *ctx) {
    (void)propagate_gray_slice(gc, ctx, INT_MAX);
}

/* markold：重扫 [firstold1, reallyold) 的 OLD1 对象（上一轮晋升、仍可能指向 young）。
   逐个 force_trace（其 young 子对象被 promote 传播提升到 OLD0），然后推进 OLD。
   firstold1 由 sweepgen 设置（首个 OLD1），有界扫描近期晋升区——无独立中年代。
   OLD1 是自限的，只存在一轮，下轮就变 OLD 不再被 minor 访问。 */
static void markold(Gc *gc, GcTraceCtx *ctx) {
    if (!gc->firstold1) return; /* 无 OLD1（sweep 未产生），跳过扫描 */
    for (Obj *o = gc->firstold1; o && o != gc->reallyold; o = o->next) {
        if (o->age == MICA_AGE_OLD1) {
            force_trace(gc, o, ctx);
            o->age = MICA_AGE_OLD; /* OLD1 属 old 代，计数不变 */
        }
    }
    gc->firstold1 = NULL;
    propagate_gray(gc, ctx);
}

/* ---------- GCv2 OPT-1/OPT-4：propagate_gray 分片 ----------

   旧 propagate_gray 一次 while 清空全部 gray 链，是大对象图（百万级 gray 节点）
   下的 O(N) 突刺源。改为 propagate_gray_slice(Gc*, GcTraceCtx*, int budget)：
   每次推进 budget 个灰对象（默认 GC_PROPAGATE_BUDGET=64），预算耗尽返回。

   调用契约：
   - mark_collection_minor：minor 仍原子，budget=INT_MAX（一次清完）——minor avg 仅 26us
   - gc_mark_slice / gc_step 的 MARKING 分支：budget=GC_MARK_OBJ_BUDGET，与 mark_field_budget
     协同（对象预算 + 字段预算双轴控制，最大大对象单 step 占 GC_PROPAGATE_BUDGET * ~256 字字段）
   - finalize 清扫前调 gc_marked_bytes += 当前 slice 实际标记字节（OPT-8）

   不变量：
   - 灰对象弹出后 mark=BLACK（force_trace 同），重复入链不可能（major_push 检查 iswhite）
   - 链表严格收缩（每次弹出一个）——总终止 */

/* GC_PROPAGATE_BUDGET：单次 propagate_gray_slice 最大处理灰对象数。64 与 GC_SWEEP_BUDGET 对齐，
   保证 step 工作量稳定（mark + sweep 每步等量分片）。 */
#ifndef GC_PROPAGATE_BUDGET
#define GC_PROPAGATE_BUDGET 64
#endif

static int propagate_gray_slice(Gc *gc, GcTraceCtx *ctx, int budget) {
    int done = 0;
    while (gc->gray && done < budget) {
        Obj *o = gc->gray;
        gc->gray = OBJ_GCLIST(o);
        OBJ_GCLIST(o) = NULL;
        trace_object(gc, o, ctx);
        o->mark = GC_BLACK_BIT;
        done++;
        /* GCv2 OPT-8：累计本周期已标记字节（替代估算）。
           ⚠️ 此处不调 obj_size（switch 在 mark 热路径开销大）——改在 collect 末尾根据
           存活字节差额近似（live_bytes_after - live_bytes_before）。注：live_bytes 来自
           alloc/free 精确累计（OPT-2），仍可用于 setpause 准确计算。 */
    }
    return done;
}

/* ---------- trace 原语（vtable 回调内使用） ---------- */

/* promote 模式（markold 传播）的入链/原子 trace：young 子对象提升到 OLD0，白→灰入 gray。
   已灰/已黑（子对象已传播或在 gray 链）直接返回。 */
static void promote_trace(Gc *gc, Obj *o, GcTraceCtx *ctx) {
    if (AGE_IS_YOUNG(o->age)) promote_young(gc, o);
    if (isgray(o) || isblack(o)) return;
    if (obj_is_grayable(gc, o)) {
        o->mark = 0;
        OBJ_GCLIST(o) = gc->gray;
        gc->gray = o;
    } else if (o->type < GC_VTABLE_SLOTS && gc->vtables[o->type].trace) {
        trace_object(gc, o, ctx);
        o->mark = GC_BLACK_BIT;
    } else {
        o->mark = GC_BLACK_BIT;
    }
}

/* 强制 promote trace（不论当前颜色）：写屏障把非 grayable 容器（capval/自定义）提升到
   OLD0 时，原子 trace 提升其 young 后代——否则该容器 OLD0+黑、minor 的 markold（只扫
   OLD1）/根 trace（跳过黑）都不重扫它，后代 white 被 sweep_young 误回收（问题 A）。
   先置灰防 self-cycle 递归，trace 后置黑。 */
static void force_promote_trace(Gc *gc, Obj *o, GcTraceCtx *ctx) {
    if (o->flags & OBJ_PERMANENT) return;
    o->mark = 0;
    if (o->type < GC_VTABLE_SLOTS && gc->vtables[o->type].trace) {
        trace_object(gc, o, ctx);
    }
    o->mark = GC_BLACK_BIT;
}

void gc_trace_value(GcTraceCtx *ctx, const Value *v) {
    if (!val_is_obj(*v)) return;
    Obj *o = val_as_obj(*v);
    if (o->flags & OBJ_PERMANENT) return;
#ifdef MICA_GC_DEBUG
    /* D10 不变量校验：代际校验（OLD 对象不得指向 young）与根覆盖校验
       （gc_check_permanent_roots：非根 permanent 对象的运行期子对象必须已被标记——
       白 = 未被任何根 trace = 登记缺口，未标记的子对象将被本轮清扫回收 → 悬挂引用）。 */
    if (MICA_UNLIKELY(ctx->check_old_young && AGE_IS_YOUNG(o->age))) {
        fprintf(stderr, "GC-DEBUG: old->young reference (target age=%d)\n", o->age);
        assert(!"old object references young object");
    }
    if (MICA_UNLIKELY(ctx->check_white_child && iswhite(ctx->gc, o))) {
        fprintf(stderr,
                "GC-DEBUG: unregistered permanent root %p(type=%d) references white runtime object %p(type=%d)\n",
                (const void *)ctx->report_source, ctx->report_source ? ctx->report_source->type : -1,
                (void *)o, o->type);
        assert(!"permanent object's runtime child not covered by any registered root");
    }
#endif
    if (ctx->promote) {
        promote_trace(ctx->gc, o, ctx);
        return;
    }
    major_push(ctx->gc, o);
}

void gc_trace_obj(GcTraceCtx *ctx, Obj **op) {
    if (op == NULL || *op == NULL) return;
    Obj *o = *op;
    if (o->flags & OBJ_PERMANENT) return;
#ifdef MICA_GC_DEBUG
    if (MICA_UNLIKELY(ctx->check_old_young && AGE_IS_YOUNG(o->age))) {
        fprintf(stderr, "GC-DEBUG: old->young reference (target age=%d)\n", o->age);
        assert(!"old object references young object");
    }
    if (MICA_UNLIKELY(ctx->check_white_child && iswhite(ctx->gc, o))) {
        fprintf(stderr,
                "GC-DEBUG: unregistered permanent root %p(type=%d) references white runtime object %p(type=%d)\n",
                (const void *)ctx->report_source, ctx->report_source ? ctx->report_source->type : -1,
                (void *)o, o->type);
        assert(!"permanent object's runtime child not covered by any registered root");
    }
#endif
    if (ctx->promote) {
        promote_trace(ctx->gc, o, ctx);
        return;
    }
    major_push(ctx->gc, o);
}

/* ---------- 写屏障 ---------- */

/* 把"需要本轮传播 trace 的对象"入 gray 链（写屏障提升的 child：其 young 子对象须当轮
   被传播标记，否则本轮 young 清扫会 free 掉）。叶子无 gclist 不入链，只提升即可。
   入链须设灰（mark=0）：灰 = "已在 gray 链待 trace"，promote 传播的重入检查依赖它——
   否则已在链上的对象（mark 白）会被 promote 分支再次头插，OBJ_GCLIST 被覆盖 → 链断裂，
   后续节点从 markold 丢失 → 存活对象的 young 子对象被误回收。 */
static void link_to_gray(Gc *gc, Obj *o) {
    if (obj_is_grayable(gc, o) && !isgray(o)) {
        o->mark = 0;
        OBJ_GCLIST(o) = gc->gray;
        gc->gray = o;
    }
}

/* P0 修复：把代际提升的 young 子对象挂入 promote_list（**不改 mark**）。
   与 link_to_gray 的根本区别：IDLE/REBUILD 期绝不能把活的黑对象置灰，否则
   sweepgen 的 !isblack 判据会把它当垃圾 free（且该节点仍留在 gray 链上，下一轮
   propagate 对已释放内存写 mark/trace）。promote_list 的节点保持原色，
   收集开始时由 drain_promote_list splice 进 gray 链，再由 promote 传播消费。

   幂等性：重复挂链**不会**造成误回收——对象只要不被根 trace 到又不被传播，就仍是
   young 白对象，会按正常的"未达即回收"规则被回收（那才是正确结果）；一旦被任一路径
   到达，它就会被正确标黑。重复挂会导致 gray 环序被扰动（trace 幂等，无正确性影响）。 */
static void link_to_promote(Gc *gc, Obj *o) {
    if (!obj_is_grayable(gc, o)) return;  /* 非 grayable 由 force_promote_trace 原子处理 */
    OBJ_GCLIST(o) = gc->promote_list;
    gc->promote_list = o;
}

/* 把 promote_list 整体 splice 到 gray 链头部（收集开始前调用）。
   逐个置灰（mark=0）使其可被 propagate_gray_slice 消费；已黑对象跳过（其子对象已标）。 */
static void drain_promote_list(Gc *gc) {
    while (gc->promote_list) {
        Obj *o = gc->promote_list;
        gc->promote_list = OBJ_GCLIST(o);
        if (!isblack(o) && !isgray(o)) {
            o->mark = 0;                     /* 灰：待传播 */
            OBJ_GCLIST(o) = gc->gray;
            gc->gray = o;
        } else {
            OBJ_GCLIST(o) = NULL;
        }
    }
}

/* 写屏障慢路径：IDLE promote + REBUILD/MARKING/SWEEPING 增量三色。
   热路径（IDLE + young target / 标量写）由 gc.h 的内联包装挡掉，本函数只处理
   IDLE old-target+obj 的 promote，以及收集期间的三色屏障。 */
void gc_record_write_slow(Gc *gc, Obj *target, Value v) {
    if (gc->phase == GC_IDLE) {
        if (AGE_IS_YOUNG(target->age)) return;
        if (!val_is_obj(v)) return;
        /* 代际：old target 写入 young child → 提升 child 到 OLD0 并入 gray 链 */
        Obj *child = val_as_obj(v);
        if (AGE_IS_YOUNG(child->age) &&
            !(target->flags & OBJ_PERMANENT) && !(child->flags & OBJ_PERMANENT)) {
            child->age = MICA_AGE_OLD0;
            if (obj_is_grayable(gc, child)) {
                link_to_promote(gc, child); /* 入 promote_list：收集时传播，**不改 mark** */
            } else if (child->type < GC_VTABLE_SLOTS && gc->vtables[child->type].trace) {
                /* 非 grayable 但有 trace（capval/自定义容器）：不入 gray 链，原子 promote
                   trace 保护其 young 后代（否则 OLD0+黑 容器在 minor 不被重扫 → 后代误回收） */
                GcTraceCtx pctx = { .gc = gc, .promote = true };
                force_promote_trace(gc, child, &pctx);
            } else {
                /* 叶子（string/native 等，无 gclist 无 trace）：link_to_gray 不处理 → 保持白 →
                   下个 major 清扫误 free（old 容器写 young 叶子的 UAF）。直接置黑标记存活。 */
                child->mark = GC_BLACK_BIT;
            }
            gc->stats.young_live_count--;
            gc->stats.old_live_count++;
        }
        return;
    }
    if (gc->phase == GC_REBUILD) {
        /* rebuild 期：存活对象正被分片推进 OLD。任何 target 写入 young child 都须提升——
           否则 young 代容器被置 OLD 后，其 young 后代不被 minor 重扫（markold 只扫 OLD1、
           根 trace 跳过黑容器）→ 误回收。新头插对象 stay young，其 child 提前提升无害
           （进 old 后由 major 回收）。 */
        if (!val_is_obj(v)) return;
        Obj *child = val_as_obj(v);
        if (AGE_IS_YOUNG(child->age) && !(child->flags & OBJ_PERMANENT)) {
            child->age = MICA_AGE_OLD0;
            if (obj_is_grayable(gc, child)) {
                link_to_promote(gc, child);
            } else if (child->type < GC_VTABLE_SLOTS && gc->vtables[child->type].trace) {
                GcTraceCtx pctx = { .gc = gc, .promote = true };
                force_promote_trace(gc, child, &pctx);
            } else {
                child->mark = GC_BLACK_BIT;
            }
            gc->stats.young_live_count--;
            gc->stats.old_live_count++;
        }
        return;
    }
    if (!val_is_obj(v)) return;
    Obj *child = val_as_obj(v);
    if (gc->phase == GC_MARKING) {
        /* 重置子状态（reset_cur != NULL）期间抑制向后屏障：reset 会把全链对象重置为当前白，
           播种（gc_root_trace_slice）再重新 trace 根——此时 black target 的向后屏障会把尚未
           reset 的对象链入 gray_again，随后 reset 又将其置白但未从 gray_again 摘除，
           播种再 major_push 链入 gray 会覆盖 gclist → gray/gray_again 链交错损坏 → 孤儿化误回收。 */
        /* 增量三色：黑 target 不能引用白 child。permanent 根恒视为"已标"（根）：
           写 young child 直接 push 入 gray（否则原子收尾须重扫永久根，大根 O(all) 突刺）。 */
        if (isblack(target) || (target->flags & OBJ_PERMANENT)) {
            if (!(target->flags & OBJ_PERMANENT) && obj_is_grayable(gc, target)) {
                target->mark = 0;
                OBJ_GCLIST(target) = gc->gray_again;
                gc->gray_again = target;
            } else if (iswhite(gc, child) && !(child->flags & OBJ_PERMANENT)) {
                major_push(gc, child);
            }
        } else if (isgray(target) && iswhite(gc, child) && !(child->flags & OBJ_PERMANENT)) {
            /* 灰 target（增量大容器已部分 trace、中断待续扫）：写入已 trace 槽的白子对象
               必须直接入 gray——否则续扫从中断槽开始，跳过已 trace 槽里被 compact 等原地
               搬移进来的新白子对象 → 白子对象残留 → major 清扫误回收（2026-08-15：compact
               原地写 OLD 数组触发）。灰 target 全量 trace 时会跳过已灰/已黑子对象，此提前
               push 无害（仅提前标记）。 */
            major_push(gc, child);
        }
    } else if (gc->phase == GC_SWEEPING) {
        if (iswhite(gc, child) && !(child->flags & OBJ_PERMANENT)) {
            child->mark = GC_BLACK_BIT; /* 清扫期新引用存活 */
        }
    }
}

/* ---------- unsafe 构造窗口 ---------- */

void gc_begin_unsafe(Gc *gc) {
    gc->unsafe_depth++;
}

void gc_end_unsafe(Gc *gc) {
    assert(gc->unsafe_depth > 0);
    gc->unsafe_depth--;
}

void gc_free_permanent(Gc *gc, Obj *o) {
    if (!o) return;
    /* 防御：非 permanent 对象不在 permanent/perm_strs 链上，直接 free 会把仍挂在
       all_obj 链上的对象释放掉 → 链中留下悬垂节点（宿主误把协程 context 传给
       mica_context_free 即触发）。明确报告并拒绝，而非静默破坏堆。 */
    if (!(o->flags & OBJ_PERMANENT)) {
        fprintf(stderr, "mica: gc_free_permanent on non-permanent object %p(type=%d) ignored\n",
                (void *)o, o->type);
        return;
    }
    /* 增量 major 的分片播种/根 trace 游标若正指向该对象，free 后会读悬垂 → 标记损坏。
       摘链前先把游标推进到 o->next（或清空），保证 gc_root_trace_slice 不碰已 free 内存。
       GC_ULTRA_PLAN D4 补全：reset/rebuild/标记中断游标同样处理。 */
    if (gc->root_trace_cur == o) gc->root_trace_cur = o->next;
    if (gc->root_arr_obj == o) gc->root_arr_obj = NULL;
    if (gc->reset_cur == o) gc->reset_cur = o->next;
    if (gc->rebuild_cur == o) gc->rebuild_cur = o->next;
    if (gc->mark_interrupt_obj == o) gc->mark_interrupt_obj = NULL;
    /* D2：若登记为根则摘除。**必须留 NULL 洞，不能把尾元素交换进来**：
       交换会把尾根落到已越过的下标（i < root_trace_idx），正在进行的增量 major 播种
       游标不会再访问它 → 该根的运行期子对象本轮不被标记 → 误回收（悬挂引用）。
       播种/遍历循环遇 NULL 跳过；末尾连续 NULL 收缩 n_roots 以维持完成判定。 */
    for (int i = 0; i < gc->n_roots; i++) {
        if (gc->roots[i] == o) {
            gc->roots[i] = NULL;
            while (gc->n_roots > 0 && gc->roots[gc->n_roots - 1] == NULL) gc->n_roots--;
            break;
        }
    }
    Obj **link = &gc->permanent;
    while (*link && *link != o) link = &(*link)->next;
    if (*link != o) { /* 可能在永久字符串专用链（叶子；宿主显式弃用字符串少见但允许） */
        link = &gc->perm_strs;
        while (*link && *link != o) link = &(*link)->next;
    }
    if (*link == o) *link = o->next;
    if (o->type < GC_VTABLE_SLOTS && gc->vtables[o->type].finalize) {
        gc->vtables[o->type].finalize(o);
    }
    free(o);
}

/* ---------- contexts 链 ---------- */

void gc_add_context(Gc *gc, Obj *o) {
    ((mica_context *)o)->ctx_next = gc->contexts;
    gc->contexts = o;
}

void gc_remove_context(Gc *gc, Obj *o) {
    if (!o) return;
    Obj **link = &gc->contexts;
    while (*link && *link != o) link = &((mica_context *)(*link))->ctx_next;
    if (*link == o) *link = ((mica_context *)o)->ctx_next;
    ((mica_context *)o)->ctx_next = NULL;
}

/* 收集安全点（标记完成、currentwhite 翻转、清扫开始前）：死 context 的开放捕获值若被
   存活闭包引用（黑 capval），其 p 指向将随 context 释放的栈，须先把值拷入自有 value 关闭；
   否则 obj_context_finalize 遍历 opencapval 链时，capval（新于 context、all_obj 链头侧）可能
   已被清扫 free → 读悬垂 p/open_next → UAF。关闭后整链摘除——白 capval 随 context 一起死，
   无需关闭（其值不留存）。活 context 保持 open（栈值由 context_trace 保底）。 */
static void gc_close_dead_contexts(Gc *gc) {
    for (Obj *co = gc->contexts; co; co = ((mica_context *)co)->ctx_next) {
        mica_context *c = (mica_context *)co;
        if (isblack(c)) continue; /* 活 context：capval 保持 open */
        ObjCapVal *cap = c->opencapval;
        while (cap) {
            ObjCapVal *next = cap->open_next;
            if (isblack(cap)) obj_capval_close(gc, cap);
            cap = next;
        }
        c->opencapval = NULL;
    }
}

/* ---------- 弱引用（WeakRef） ---------- */

void gc_add_weakref(Gc *gc, ObjWeakRef *r) {
    r->next_weakref = (ObjWeakRef *)gc->weakrefs;
    gc->weakrefs = (Obj *)r;
}

/* collect_age：本收集清扫的最大 age（6 态；young=1(SURV)/major=TOUCHED=哨兵"全部"）。
   更老代不扫，恒存活。 */
static bool weakref_alive(Gc *gc, Obj *o, int collect_age) {
    (void)gc;
    if (o->flags & OBJ_PERMANENT) return true;
    /* OLD0 对象由写屏障提升后物理上仍在 young 区，sweepgen 会清扫白色的 OLD0（free）。
       只有 OLD1/OLD/TOUCHED 才"恒不被 minor 清扫"，可免 isblack 检查直接判活；
       OLD0 须与 NEW/SURV 一样用 isblack 判定——否则死 OLD0 目标被误判存活 →
       weakref.target 悬垂 → 解引用 UAF。major（collect_age=TOUCHED）下 age>6 恒假，全走 isblack。 */
    if ((int)o->age > collect_age + 1) return true;
    return isblack(o);
}

/* GC_ULTRA_PLAN D6 双链弱引用清理：
   - minor（collect_age<=SURV）：只扫 young 目标链（weakrefs）。条目目标 age>=OLD1 时
     （minor 永不清扫该代，weakref_alive 的 age 短路恒真）迁移到 old 链（weakrefs_old）——
     之后 minor 不再检查它。
   - major（collect_age=TOUCHED）：两链都扫。old 链条目目标此时可能已死（major 全量清扫），
     存活目标留 old 链（major 后全部对象置 OLD，无需迁回）。
   正确性：迁移条件（target age >= OLD1）⟹ 目标在 minor 不被清扫（sweep_young 只扫
   [all_obj, old1) 区）⟹ 无需在 minor 检查。weakref 对象自身存活判据不变。 */
void gc_clear_weakrefs(Gc *gc, int collect_age) {
    const bool is_minor = (collect_age != MICA_AGE_TOUCHED);
    /* young 目标链：minor 扫 + 老目标迁移；major 全扫（老目标此时可能已死）。 */
    ObjWeakRef **link = (ObjWeakRef **)&gc->weakrefs;
    while (*link) {
        ObjWeakRef *r = *link;
        if (!weakref_alive(gc, (Obj *)r, collect_age)) {
            *link = r->next_weakref; /* weakref 自身死：摘除 */
            continue;
        }
        if (val_is_obj(r->target)) {
            Obj *t = val_as_obj(r->target);
            if (!weakref_alive(gc, t, collect_age)) {
                r->target = val_null(); /* 目标死：清 null */
                link = &r->next_weakref;
                continue;
            }
            /* D6 迁移：目标 age>=OLD1（minor 不扫该代）且 weakref 自身已 old
               （young weakref 自身存活须每 minor 检查，故不迁移；迁后由 major 判活） */
            if (is_minor && t->age >= MICA_AGE_OLD1 && ((Obj *)r)->age >= MICA_AGE_OLD1) {
                ObjWeakRef *nx = r->next_weakref;
                *link = nx;
                r->next_weakref = (ObjWeakRef *)gc->weakrefs_old;
                gc->weakrefs_old = (Obj *)r;
                continue;
            }
        }
        link = &r->next_weakref;
    }
    if (!is_minor) {
        /* major：old 链全扫；死条目摘除，存活留链（major 后全部对象 OLD） */
        ObjWeakRef **link2 = (ObjWeakRef **)&gc->weakrefs_old;
        while (*link2) {
            ObjWeakRef *r = *link2;
            if (!weakref_alive(gc, (Obj *)r, collect_age)) {
                *link2 = r->next_weakref;
                continue;
            }
            if (val_is_obj(r->target) && !weakref_alive(gc, val_as_obj(r->target), collect_age)) {
                r->target = val_null();
            }
            link2 = &r->next_weakref;
        }
    }
}

/* ---------- 标记（minor 真分代） ---------- */

#ifdef MICA_GC_DEBUG
/* ============================================================
   GC_ULTRA_PLAN D10：不变量校验（仅 MICA_GC_DEBUG 构建；发布零开销）。
   调用点：gc_collect_minor 尾部（灰链完整性 + 代际不变量）、gc_collect_major 尾部
   （灰链完整性 + 根登记完整性）。
   ============================================================ */

/* 灰链完整性：gray/gray_again 为空时 all_obj 上不得有灰对象（mark==0）。
   GC_BUG2 孤儿化检测的自动版（当时手工打点定位：孤儿对象被清扫误 free）。 */
static void gc_check_orphan_gray(Gc *gc) {
    if (gc->gray || gc->gray_again) return;
    for (Obj *o = gc->all_obj; o; o = o->next) {
        if (isgray(o)) {
            fprintf(stderr, "GC-DEBUG: orphan gray obj %p type=%d age=%d mark=%d\n",
                    (void *)o, o->type, o->age, o->mark);
            assert(!"orphan gray object");
        }
    }
}

/* 代际不变量：minor 后 OLD 对象（age==OLD，[reallyold, ∞) 段）不得指向 young（NEW/SURV）。
   以 check_old_young 标志重扫——存活子对象已被 promote 提升到 OLD0+（黑），
   major_push 对非白对象 no-op，校验无副作用；真违反（young 子对象）即断言。 */
static void gc_check_old_no_young(Gc *gc) {
    GcTraceCtx vctx = { .gc = gc, .promote = false, .check_old_young = true };
    for (Obj *o = gc->reallyold; o; o = o->next) {
        if (o->age == MICA_AGE_OLD && o->type < GC_VTABLE_SLOTS &&
            gc->vtables[o->type].trace) {
            gc->vtables[o->type].trace(o, &vctx);
        }
    }
}

/* 根覆盖完整性（D2）：收集标记完成后，重扫 permanent 链上未登记根——其运行期子对象
   必须是黑（被某登记根 trace 覆盖，如命名空间 map 经 globals）；白子对象 = 未被任何根
   标记 = 登记缺口 → 断言。合法场景：未登记 permanent 的引用被登记根（globals 等）
   传递覆盖 → 子对象黑 → 通过。真缺口：子对象白 → 本轮清扫将回收 → 悬挂引用。
   调用点须在"存活=黑、死=白"的标记完成态（minor 翻白前 / major rebuild 完成后）。
   副作用：白子对象会被 major_push 标记（灰），但断言立即终止进程，无实际影响。 */
static void gc_check_permanent_roots(Gc *gc) {
    if (!gc->debug_checks_enabled) return; /* 负例测试临时抑制 */
    GcTraceCtx pctx = { .gc = gc, .promote = false, .check_white_child = true };
    for (Obj *o = gc->permanent; o; o = o->next) {
        if (gc_is_root(gc, o)) continue; /* 登记根：正常 trace 路径已覆盖 */
        if (o->type < GC_VTABLE_SLOTS && gc->vtables[o->type].trace) {
            pctx.report_source = o; /* 仅报告用（断言消息标注源永久根） */
            gc->vtables[o->type].trace(o, &pctx);
            pctx.report_source = NULL;
        }
    }
}
#endif /* MICA_GC_DEBUG */

/* minor 标记：markold（有界重扫近期晋升 OLD1 + promote 传播）→ 根（permanent）→ contexts 链
   （old 协程栈每轮重扫）→ 传播。不碰 old 段对象（promote 已保证不指向 young）。 */
static void mark_collection_minor(Gc *gc) {
    GcTraceCtx mctx = { .gc = gc, .promote = true };
    markold(gc, &mctx);
    /* 写屏障提升的 OLD0 容器（promote_list）须 promote 传播提升其 young 后代：markold 只扫
       OLD1，无 OLD1 时该链仍含 OLD0 容器；非 promote 传播让后代保持 young 而 OLD0 容器
       黑、minor 根 trace 跳过 → 后代 white 被 sweepgen 误回收（问题 A）。 */
    drain_promote_list(gc);
    propagate_gray(gc, &mctx);

    /* 永久根子对象用 promote 模式 trace：存活的 young 子对象提升到 OLD0（2 轮老化），
       避免永久根每轮全量重扫（大永久根持有大量存活对象如全局数组的 GC 开销——闭包/容器
       场景吞吐瓶颈）。被丢弃的子对象（minor 前已从根移除）不被 trace → 不提升 → 保持
       young 由 minor 回收（与写屏障提升不同——写屏障在写入时提升，丢弃的对象也滞留）。
       GC_ULTRA_PLAN D2：explicit_roots=true 时只 trace 登记根（消灭 O(permanent 全链)
       固定成本）；false 时 trace 全 permanent 链（现状行为）。 */
    GcTraceCtx pctx = { .gc = gc, .promote = true };
    if (gc->explicit_roots) {
        for (int i = 0; i < gc->n_roots; i++) {
            if (gc->roots[i]) trace_object(gc, gc->roots[i], &pctx);
        }
    } else {
        for (Obj *o = gc->permanent; o; o = o->next) {
            trace_object(gc, o, &pctx);
        }
    }
    /* contexts：age>=OLD0（old）的存活协程栈值每轮重扫（young 由根传播经
       contexts 链覆盖；isblack 排除死 context——否则其栈值被复活泄漏到 major）。
       协程栈值生命周期短，不用 promote（不提前老化）。 */
    GcTraceCtx ctx = { .gc = gc, .promote = false };
    for (Obj *o = gc->contexts; o; o = ((mica_context *)o)->ctx_next) {
        if (o->age >= MICA_AGE_OLD0 && isblack(o) && o->type < GC_VTABLE_SLOTS &&
            gc->vtables[o->type].trace) {
            gc->vtables[o->type].trace(o, &ctx);
        }
    }
    propagate_gray(gc, &ctx);
}

/* ---------- 清扫（minor：sweepgen 两段） ---------- */

/* GC_ULTRA_PLAN D8：批量 finalize+free（片内收集、片尾统一释放——free 局部性 + 为
   freelist 留接口）。finalize 顺序保持链序（死对象按扫到顺序收集）。调用方在片尾 flush：
   死对象计数满 GC_DEAD_BATCH 时先 flush 再继续。 */
#define GC_DEAD_BATCH 64
static void gc_free_batch(Gc *gc, Obj **dead, int n) {
    for (int i = 0; i < n; i++) {
        Obj *o = dead[i];
        const int dage = o->age; /* free 前捕获（free 后读属 use-after-free） */
        const size_t dsize = obj_size(o); /* GCv2 OPT-2：精确字节记账（free 前捕获 size） */
        if (o->type < GC_VTABLE_SLOTS && gc->vtables[o->type].finalize) {
            gc->vtables[o->type].finalize(o);
        }
        free(o);
        if (AGE_IS_YOUNG(dage)) gc->stats.young_live_count--;
        else gc->stats.old_live_count--; /* OLD0 已属 old */
        /* GCv2 OPT-2：从 live_bytes 减去（obj_size 已包含容器动态部分） */
        gc->live_bytes -= (int64_t)dsize;
    }
}

/* sweepgen：清扫 [*p, limit) 区域（Lua 5.5）。死（非黑）= 批量收集（片尾 finalize+free 摘除）、
   按实际 age 递减 young/old 计数；活 = 原位推进 age（NEW→SURV 重置白、SURV→OLD1、OLD0→OLD1），
   不重链（边界指针原位移动）。返回清扫后 *p 位置。firstold1 记录本区首个 OLD1
   （markold 有界扫描起点）。OLD1/OLD 理论上不在 young 区域（在 [old1,reallyold)）。
   D8：代龄推进查表（Lua nextage，lgc.c:1175-1183）；记账语义保留现状
   （SURV→OLD1 记 gc_marked_old 驱动自适应 major；OLD0→OLD1 只计数不记账）。

   ⚠️ 存活判据 = isblack || isgray（**不能只用 isblack**）。灰对象是"已在 gray/promote
   链上待传播"的对象，仍被某条链持有；过去的 `!isblack ⇒ 死` 会在"清扫前新挂入 gray"
   的窗口（gc_close_dead_contexts 关闭 capval 触发写屏障、清扫期 finalizer 内的写入）
   把活对象误 free，并且该节点仍留在 gray 链上 → 下一轮 propagate 对已释放内存写 mark。
   判 isgray 为活最多让一个对象多活一轮，由下一轮传播或清扫自然回收（安全方向的偏差）。 */
/* 代龄推进查表（Lua nextage，lgc.c:1175-1183）：
   NEW→SURV（颜色由调用方 resetwhite 处理）、SURV→OLD1、OLD0→OLD1、OLD1/OLD→OLD、
   TOUCHED 为哨兵不推进。OLD1/OLD 理论上不在 young 区域（在 [old1,reallyold)）。 */
static const uint8_t k_next_age[6] = {
    MICA_AGE_SURV,     /* NEW    → SURV */
    MICA_AGE_OLD1,     /* SURV   → OLD1 */
    MICA_AGE_OLD1,     /* OLD0   → OLD1 */
    MICA_AGE_OLD,      /* OLD1   → OLD（不在 young 区，实际不经过） */
    MICA_AGE_OLD,      /* OLD    → OLD（保持） */
    MICA_AGE_TOUCHED,  /* TOUCHED →（哨兵，实际不经过） */
};

static Obj **sweepgen(Gc *gc, Obj **p, Obj *limit) {
    Obj *dead_list[GC_DEAD_BATCH];
    int dead_n = 0;
    while (*p != NULL && *p != limit) {
        Obj *o = *p;
        Obj *next = o->next;
        if (isblack(o) || isgray(o)) {  /* 活（黑=已标；灰=在待传播链上，本轮不移除） */
            if (o->age == MICA_AGE_NEW) {
                resetwhite(gc, o);
                o->age = MICA_AGE_SURV;   /* 存活留 young，重置当前白 */
            } else {
                const uint8_t na = k_next_age[o->age];
                if (na == MICA_AGE_OLD1) {
                    if (o->age == MICA_AGE_SURV) {
                        /* 仅 SURV→OLD1 迁移计数（OLD0 在写屏障提升时已 young--/old++，
                           此处不得重复迁移——否则 young/old 存活计数损坏）。 */
                        gc->stats.young_live_count--;
                        gc->stats.old_live_count++;
                        gc->gc_marked_old += obj_size(o); /* 记自适应账（驱动 major） */
                    }
                    /* OLD0→OLD1：只推进 age + 记 firstold1，不记账（现状语义） */
                    if (gc->firstold1 == NULL) gc->firstold1 = o;
                }
                o->age = na;
            }
            p = &o->next;
        } else {  /* 死：批量收集（D8，片尾统一 finalize+free） */
            *p = next;
            dead_list[dead_n++] = o;
            if (dead_n == GC_DEAD_BATCH) {
                gc_free_batch(gc, dead_list, dead_n);
                dead_n = 0;
            }
        }
    }
    if (dead_n > 0) gc_free_batch(gc, dead_list, dead_n);
    return p;
}

/* 清扫 young 代（NEW + SURV 区域 [all_obj, old1)），原位推进 age 并 O(1) 更新边界指针：
   reallyold=旧 old1、old1=首个 SURV 幸存、survival=all_obj。old 代（[old1,?)）不碰。 */
static void sweep_young(Gc *gc) {
    Obj **psurvival = sweepgen(gc, &gc->all_obj, gc->survival);
    sweepgen(gc, psurvival, gc->old1);
    gc->reallyold = gc->old1;
    gc->old1 = *psurvival ? *psurvival : gc->reallyold;
    gc->survival = gc->all_obj;
}

/* ---------- young 收集（原子） ---------- */

void gc_collect_minor(Gc *gc) {
    if (!gc->enabled || gc->unsafe_depth > 0 || gc->phase != GC_IDLE) return;
    /* D4：重入防护——收集嵌套（collect_depth>0，如 finalizer 在 minor 清扫内直接调收集）
       直接返回。step_depth 不检查：gc_step 的 IDLE 分支合法调用（step_depth==1）；
       finalizer 内分配 → gc_alloc → gc_step 已被 step_depth 挡（不会经此路径重入 minor）。 */
    if (MICA_UNLIKELY(gc->collect_depth > 0)) return;
    gc->collect_depth++;

    double t0 = gc->stats_enabled ? mica_now_us() : 0;
    gc->stats.minor_count++;
    gc->stats.round++;
    mark_collection_minor(gc);
#ifdef MICA_GC_DEBUG
    gc_check_permanent_roots(gc); /* 标记完成态（存活=黑）：根覆盖缺口在此暴露 */
#endif
    gc->currentwhite = otherwhite(gc); /* 标记完成 → 翻转 */
    gc_clear_weakrefs(gc, MICA_AGE_SURV);
    gc_close_dead_contexts(gc); /* 清扫前关闭死 context 的黑 capval，防 finalize 读已 free capval */
    /* 兜底：标记期间若有 finalizer/capval close 之类再次写入 old 容器（写屏障 IDLE 路径
       把 young 子对象提升到 OLD0 并挂入 promote_list），在清扫前再传播一次——
       否则该子对象的 young 后代不被提升，本轮清扫会误回收。 */
    if (MICA_UNLIKELY(gc->promote_list != NULL)) {
        GcTraceCtx pc2 = { .gc = gc, .promote = true };
        drain_promote_list(gc);
        propagate_gray(gc, &pc2);
    }
    sweep_young(gc);
    gc->minor_bytes_base = gc->alloc_bytes; /* 字节触发基数重置（统计保留） */
    /* D9：setminordebt 等价（Lua lgc.c:1417-1418）——债务 = 总存活近似字节 × MINORMUL%。
       GCv2 OPT-2：用 live_bytes 精确值（替代 `(old+young)*64` 估算——大对象场景估算偏差大）。
       下限 GC_YOUNG_BYTES：防小堆病态频繁（每分配即 minor）。 */
    {
        gc->debt = (ptrdiff_t)(gc->live_bytes * GC_MINORMUL / 100);
        if (gc->debt < (ptrdiff_t)GC_YOUNG_BYTES) gc->debt = (ptrdiff_t)GC_YOUNG_BYTES;
    }
#ifdef MICA_GC_DEBUG
    gc_check_orphan_gray(gc);
    gc_check_old_no_young(gc);
#endif
    gc->collect_depth--;
    if (gc->stats_enabled) {
        double us = mica_now_us() - t0;
        record_pause(gc, us);
        gc->stats.minor_total_us += us;
        if (us > gc->stats.minor_max_us) gc->stats.minor_max_us = us;
        gc->stats.minor_n++;
    }
}

/* ---------- major（增量） ---------- */

/* 增量 reset 片：把全链对象分片重置为当前白（消除 gc_begin_major 的 O(all) 起始突刺）。
   reset 期新头插对象已是当前白（gc_alloc IDLE 分支），且位于 reset_cur 之前、本周期不处理。
   完成后立即播种根（major 从全白开始的语义不变）。 */
static void gc_reset_slice(Gc *gc) {
    int budget = GC_SWEEP_BUDGET;
    while (gc->reset_cur && budget-- > 0) {
        Obj *o = gc->reset_cur;
        resetwhite(gc, o);
        gc->reset_cur = o->next;
    }
    if (gc->reset_cur == NULL) {
        /* 播种根改分片：设根集游标（显式根集 = 根数组下标；否则 = permanent 链），
           由 gc_root_trace_slice 分片入 gray。
           原一次性 trace_roots 对大根数组是 O(all) 单片突刺。
           清空 gray_again：reset 期间向后屏障链入的（尚未 reset 的 black）target 现已被置白，
           残留链会在播种 major_push 时被覆盖 gclist → gray/gray_again 链交错损坏 → 孤儿化误回收。
           （播种会从根重新 trace 所有存活对象，gray_again 残留无需保留。） */
        gc->gray_again = NULL;
        gc->root_arr_obj = NULL;
        if (gc->explicit_roots) {
            gc->root_trace_idx = 0; /* D2：根数组游标（gc_root_trace_slice 消费） */
            gc->root_trace_cur = NULL;
        } else {
            gc->root_trace_cur = gc->permanent;
            gc->root_trace_idx = 0;
        }
    }
}

void gc_begin_major(Gc *gc) {
    /* D4：finalizer 在 minor 清扫内直接调 gc_begin_major → 收集嵌套（collect_depth>0）
       直接返回（否则 minor 清扫中途 phase 变 MARKING → 状态机损坏）。 */
    if (MICA_UNLIKELY(gc->collect_depth > 0)) return;
    gc->stats.major_count++;
    gc->gray = NULL;
    gc->gray_again = NULL;
    /* promote_list 不清：其中对象是 young（可能未被本轮根 trace 覆盖），
       它们的 mark 未被改动，若本轮未被标记就是垃圾 → 正常清扫规则回收；
       若被标记则已黑。清链会让链上的 gclist 变成悬垂指针（对象仍在 all_obj），
       故这里只把链摘下（节点由各自的 mark 决定生死）。 */
    gc->promote_list = NULL;
    gc->mark_interrupt_obj = NULL;
    gc->mark_interrupted = false;
    /* GCv2 OPT-8：重置本周期已标记字节计数器（propagate_gray_slice 累计） */
    gc->gc_marked_bytes = 0;
    /* major 从全白开始：增量重置 all_obj 为当前白（消除 middle/old 段残留 black 的 iswhite
       false 漏标），分片消除 O(all) 起始突刺。reset 作为 MARKING 子状态（reset_cur != NULL），
       由 gc_step 先推进 reset、后标记。 */
    gc->reset_cur = gc->all_obj;
    gc->phase = GC_MARKING;
}

/* GC_ULTRA_PLAN D7：elements-kind 跳过——NUM/BOOL 数组无 GC 对象元素，零扫描。
   ⚠️ 前提（已审计）：所有数组写点维护 kind——obj_array_set/push（obj.c）、
   解释器 INDEX_SET/SETI 快路径（vm.c）均调用 obj_array_update_kind + gc_record_write。
   kind 为 NUM/BOOL ⟹ 无对象元素 ⟹ 跳过安全。 */
static void gc_trace_array_slice(Gc *gc, Obj *obj, GcTraceCtx *ctx) {
    ObjArray *a = (ObjArray *)obj;
    if (MICA_LIKELY(a->elements_kind == ARR_ELEM_NUM ||
                    a->elements_kind == ARR_ELEM_BOOL)) {
        return; /* D7a：全 number/bool：零扫描（EMPTY 由 count==0 自然跳过） */
    }
    size_t i = 0;
    if (gc->mark_interrupt_obj == obj) {
        i = gc->mark_interrupt_slot;
        gc->mark_interrupt_obj = NULL;
    }
#if MICA_GC_SIMD && (defined(__SSE2__) || defined(_M_X64))
    /* D7b：SSE2 批量 itype——每 4 个 Value 一组（2×XMM，32B）。
       判定原理（与标量 MICA_TVISNUM 逐位等价，已推演验证）：
       itype(v) = (uint32_t)((int64_t)v >> 47)。psrlq(47) 后每 64 位通道 = v 的 bit47..63
       （17 位无符号值 V，存于通道低 32 位；高 32 位恒 0）。标量判定
       itype < MICA_TISNUM(0xFFFFFFF6) ⟺ V < 0x1FFF6：
       - number：V = bit63(符号) + 指数(11) + 尾数高 5 位 ≤ 0x1FFDF（-inf）< 0x1FFF6 ✓
         （含负 NaN：V = 0x1FFF0，同样 < 0x1FFF6 → 判 number ✓）
       - boxed：最低 tag TOBJ → V = 0x1FFF7 ≥ 0x1FFF6 → 判对象 ✓
       ⚠️ movemask 提取：cmplt_epi32 作用于全部 8 个 32 位通道——每 64 位通道的高 32 位
       （恒 0 < bound）总是真。值通道的符号位在 movemask 的位 0/2（低 32 位），须重组：
       4 值掩码 = lo位0 | lo位2>>1 | hi位0<<2 | hi位2<<1。位=1 → number（跳过）；0 → 候选
       对象 → 逐个 gc_trace_value（内部精确过滤，语义与标量一致）。预算按 4 元素计 4 字段。
       k_bound 必须与 MICA_TISNUM 保持一致（0x1FFF6 = 0xFFFFFFF6 >> 16）。 */
    const __m128i k_bound = _mm_set1_epi32(0x1FFF6);
    for (; i + 3 < a->count; i += 4) {
        __m128i lo = _mm_loadu_si128((const __m128i *)&a->items[i]);
        __m128i hi = _mm_loadu_si128((const __m128i *)&a->items[i + 2]);
        lo = _mm_srli_epi64(lo, 47);               /* 每 64 位通道：V = v>>47（逻辑） */
        hi = _mm_srli_epi64(hi, 47);
        /* 无符号比较 V < 0x1FFF0 用有符号等价（V 恒非负 0..0x1FFFF） */
        __m128i m_lo = _mm_cmplt_epi32(lo, k_bound);
        __m128i m_hi = _mm_cmplt_epi32(hi, k_bound);
        const int l = _mm_movemask_ps(_mm_castsi128_ps(m_lo));
        const int h = _mm_movemask_ps(_mm_castsi128_ps(m_hi));
        /* 重组 4 值掩码（每 64 位通道低 32 位符号位 = 位 0/2）：
           bit0=lo位0(V0)、bit1=lo位2(V1)、bit2=hi位0(V2)、bit3=hi位2(V3) */
        int mask = (l & 1) | ((l & 4) >> 1) | ((h & 1) << 2) | ((h & 4) << 1);
        if (MICA_LIKELY(mask == 0xF)) {            /* 全 number：零分支跳过 */
            if ((gc->mark_field_budget -= 4) <= 0) {
                if (i + 4 < a->count) {
                    gc->mark_interrupt_obj = obj;
                    gc->mark_interrupt_slot = i + 4;
                    gc->mark_interrupted = true;
                }
                return;
            }
            continue;
        }
        for (int k = 0; k < 4; k++) {
            if (!(mask & (1 << k))) {
                gc_trace_value(ctx, &a->items[i + k]); /* 候选对象（内部精确过滤） */
            }
            if (--gc->mark_field_budget <= 0) {
                if (i + k + 1 < a->count) {
                    gc->mark_interrupt_obj = obj;
                    gc->mark_interrupt_slot = i + k + 1;
                    gc->mark_interrupted = true;
                }
                return;
            }
        }
    }
    for (; i < a->count; i++) {                    /* 尾部标量（count % 4） */
        gc_trace_value(ctx, &a->items[i]);
        if (--gc->mark_field_budget <= 0) {
            if (i + 1 < a->count) {
                gc->mark_interrupt_obj = obj;
                gc->mark_interrupt_slot = i + 1;
                gc->mark_interrupted = true;
            }
            return;
        }
    }
#else
    for (; i < a->count; i++) {                    /* 标量路径（无 SIMD 构建） */
        gc_trace_value(ctx, &a->items[i]);
        if (--gc->mark_field_budget <= 0) {
            if (i + 1 < a->count) {
                gc->mark_interrupt_obj = obj;
                gc->mark_interrupt_slot = i + 1;
                gc->mark_interrupted = true;
            }
            return;
        }
    }
#endif
}

static void gc_trace_map_slice(Gc *gc, Obj *obj, GcTraceCtx *ctx) {
    ObjMap *m = (ObjMap *)obj;
    size_t i = 0;
    if (gc->mark_interrupt_obj == obj) {
        i = gc->mark_interrupt_slot;
        gc->mark_interrupt_obj = NULL;
    }
    for (; i < m->count; i++) {
        gc_trace_obj(ctx, (Obj **)&m->entries[i].key); /* 键是 GC 对象 */
        gc_trace_value(ctx, &m->entries[i].value);
        if ((gc->mark_field_budget -= 1) <= 0) {
            if (i + 1 < m->count) {
                gc->mark_interrupt_obj = obj;
                gc->mark_interrupt_slot = i + 1;
                gc->mark_interrupted = true;
            }
            return;
        }
    }
}

/* 分片播种：reset 完成后把根的子对象分片入 gray（大数组/表跨多片），
   替代一次性 trace_roots——大根（如 1M 元素全局数组）O(all) 单片 trace 是
   adaptive major 播种/收尾的突刺来源。播种只把子对象入 gray，gray 链由 gc_mark_slice
   分片排空（原子收尾前 gray 必然排空，不漏标）。
   关键不变量：**播种必须覆盖全部根**（含链尾 globals）——普通根（字符串/
   类/表）原子 trace 且不消耗单片预算，保证一次播种走完根集；只有大容器（array/map）
   分片 trace（单片预算限制），大根跨多片续 trace。
   GC_ULTRA_PLAN D2：explicit_roots=true 时根集 = roots[]（下标游标 root_trace_idx）；
   false 时根集 = 全 permanent 链（游标 root_trace_cur，现状行为）。 */
static void gc_root_trace_slice(Gc *gc) {
    gc->mark_field_budget = GC_MARK_FIELD_BUDGET;
    gc->mark_interrupted = false;
    GcTraceCtx ctx = { .gc = gc, .promote = false };
    if (gc->root_arr_obj) {
        /* 续上次中断的大容器 trace */
        Obj *o = gc->root_arr_obj;
        if (o->type == OBJ_ARRAY) gc_trace_array_slice(gc, o, &ctx);
        else gc_trace_map_slice(gc, o, &ctx);
        if (gc->mark_interrupted) return;
        gc->root_arr_obj = NULL;
    }
    if (gc->explicit_roots) {
        /* D2：根数组遍历（根数 ≤ GC_ROOTS_MAX，单片可走完；大容器经 root_arr_obj 中断续扫）。
           ⚠️ 完成标记 = root_trace_idx 置 n_roots（"越过末尾"），**绝不能置 0**——
           置 0 会让 gc_step/gc_collect_major 的 `idx < n_roots` 恒真 → 播种永不完成 →
           major 卡在 MARKING 无限循环（2026-08-16 修复）。若 major 中途新增根
           （n_roots 增长），idx < n_roots 会自然续播种新根，语义正确。 */
        while (gc->root_trace_idx < gc->n_roots) {
            Obj *o = gc->roots[gc->root_trace_idx++];
            if (o == NULL) continue; /* 已摘除的根（gc_free_permanent 留洞） */
            if (o->type == OBJ_ARRAY) {
                gc_trace_array_slice(gc, o, &ctx);
                if (gc->mark_interrupted) { gc->root_arr_obj = o; return; }
                if (--gc->mark_field_budget <= 0) return;
            } else if (o->type == OBJ_MAP) {
                gc_trace_map_slice(gc, o, &ctx);
                if (gc->mark_interrupted) { gc->root_arr_obj = o; return; }
                if (--gc->mark_field_budget <= 0) return;
            } else {
                trace_object(gc, o, &ctx);
            }
        }
        gc->root_trace_idx = gc->n_roots; /* 完成：越过末尾（gc_step 以 idx < n_roots 判续） */
    } else {
        while (gc->root_trace_cur) {
            Obj *o = gc->root_trace_cur;
            gc->root_trace_cur = o->next;
            if (o->type == OBJ_ARRAY) {
                gc_trace_array_slice(gc, o, &ctx);
                if (gc->mark_interrupted) { gc->root_arr_obj = o; return; }
                if (--gc->mark_field_budget <= 0) return;
            } else if (o->type == OBJ_MAP) {
                gc_trace_map_slice(gc, o, &ctx);
                if (gc->mark_interrupted) { gc->root_arr_obj = o; return; }
                if (--gc->mark_field_budget <= 0) return;
            } else {
                trace_object(gc, o, &ctx);
            }
        }
        gc->root_trace_cur = NULL;
    }
    gc->root_arr_obj = NULL;
}

static void gc_mark_slice(Gc *gc, size_t budget) {
    const bool unlimited = (budget == SIZE_MAX);
    gc->mark_field_budget = unlimited ? INT_MAX : (int)budget;
    gc->mark_interrupted = false;
    
    GcTraceCtx ctx = { .gc = gc, .promote = false };
    
    size_t obj_limit = unlimited ? SIZE_MAX : GC_MARK_OBJ_BUDGET;
    while (obj_limit-- > 0 && gc->mark_field_budget > 0 && gc->gray) {
        Obj *o = gc->gray;
        gc->gray = OBJ_GCLIST(o);

        OBJ_GCLIST(o) = NULL;
        if (o->type == OBJ_ARRAY) {
            gc_trace_array_slice(gc, o, &ctx);
        } else if (o->type == OBJ_MAP) {
            gc_trace_map_slice(gc, o, &ctx);
        } else {
            trace_object(gc, o, &ctx);
        }
        if (gc->mark_interrupted) {
            OBJ_GCLIST(o) = gc->gray;
            gc->gray = o;
            break;
        }
        o->mark = GC_BLACK_BIT;
    }
    if (gc->gray == NULL) {
        if (gc->gray_again) {
            gc->gray = gc->gray_again;
            gc->gray_again = NULL;
        } else {
            /* 原子收尾：重扫 contexts（协程栈 resume 可能无屏障写 young）。
               永久根已由 reset 播种 + MARKING 期写屏障保护，无需重扫（大根免 O(all) 突刺）。 */
            gc->mark_field_budget = INT_MAX;
            gc->mark_interrupted = false;
            GcTraceCtx rctx = { .gc = gc, .promote = false };
            for (Obj *co = gc->contexts; co; co = ((mica_context *)co)->ctx_next) {
                if (isblack(co) && co->type < GC_VTABLE_SLOTS && gc->vtables[co->type].trace) {
                    gc->vtables[co->type].trace(co, &rctx);
                }
            }
            if (gc->gray == NULL && gc->gray_again == NULL) {

                gc_clear_weakrefs(gc, MICA_AGE_TOUCHED);
                gc->currentwhite = otherwhite(gc); /* 标记完成 → 翻转 */
                gc->phase = GC_SWEEPING;
                gc_close_dead_contexts(gc); /* 清扫前关闭死 context 的黑 capval（SWEEPING 屏障安全） */
                gc->sweepgc = NULL; /* 从链头开始增量清扫 */
            }
        }
    }
}

/* major 增量清扫：每片最多 GC_SWEEP_BUDGET 个对象。死（非黑）= 批量 finalize+free 摘除
   （D8：gc_free_batch，free 局部性）；活 = resetwhite 保留原位（mark 最终由增量 rebuild
   置黑）。清扫期新对象（head 插入、置黑）在 sweepgc 之前，本片不扫，rebuild 时归位——
   增量一周期滞后。 */
/* D1：setpause 等价（Lua lgc.c:1122-1127，PAUSE=250）——major 完成后债务 = 总存活
   近似字节 × 250%（debt 为正 = 等待分配；即堆再增长约 150% 存活字节后才启动下周期）。
   下限 GC_YOUNG_BYTES 防小堆频繁 major。增量完成（gc_rebuild_slice）与显式
   gc_collect_major 共用。 */
static void gc_set_pause_debt(Gc *gc) {
    /* GCv2 OPT-2：setpause 用 live_bytes 精确值（替代原 `(old+young)*64` 估算）。
       公式 = live_bytes * PAUSE/100 - alloc_bytes（正数 = 等待分配字节）——
       等价 Lua lgc.c:1122-1127 (setpause) 但用精确字节基线。 */
    int64_t threshold = gc->live_bytes * GC_PAUSE / 100;
    int64_t used = (int64_t)gc->alloc_bytes;
    gc->debt = (ptrdiff_t)(threshold - used);
    if (gc->debt < (ptrdiff_t)GC_YOUNG_BYTES) gc->debt = (ptrdiff_t)GC_YOUNG_BYTES;
}

static void gc_rebuild_slice(Gc *gc);
static void gc_sweep_slice(Gc *gc) {
    Obj **link = gc->sweepgc ? gc->sweepgc : &gc->all_obj;
    gc->sweepgc = NULL;
    int budget = GC_SWEEP_BUDGET;
    Obj *dead_list[GC_DEAD_BATCH];
    int dead_n = 0;
    while (*link != NULL && budget-- > 0) {
        Obj *o = *link;
        if (isblack(o)) {
            resetwhite(gc, o); /* 存活：清色留原位（增量 rebuild 统一置黑/推进 OLD） */
            link = &o->next;
        } else {
            Obj *next = o->next;
            *link = next;
            dead_list[dead_n++] = o;
            if (dead_n == GC_DEAD_BATCH) {
                gc_free_batch(gc, dead_list, dead_n);
                dead_n = 0;
            }
        }
    }
    if (dead_n > 0) gc_free_batch(gc, dead_list, dead_n);
    if (*link == NULL) {
        /* 清扫完成：进入增量 rebuild（把存活对象分片推进 OLD，消除 O(all) 收尾突刺）。
           rebuild_cur 从头快照（=清扫完成时链头，含清扫期头插）开始，新对象在 rebuild
           期头插在 rebuild_cur 之前、本周期不处理（保持 young）。 */
        gc->rebuild_cur = gc->all_obj;
        gc->rebuild_first_old = gc->all_obj; /* 首个 OLD 存活 = 清扫完成时链头 */
        gc->rebuild_bytes = 0;
        gc->sweepgc = NULL;
        gc->phase = GC_REBUILD;
    } else {
        gc->sweepgc = link;
    }
}

/* major 清扫完成后的增量 rebuild 片：把存活对象分片推进 OLD（age4）+ 置黑，并增量维护
   存活计数（young → old）。完成后 O(all) 的一次性重建被多片摊薄，无收尾突刺。
   rebuild 期新头插对象（在 rebuild_cur 之前）保持 young，由后续 minor 处理。 */
static void gc_rebuild_slice(Gc *gc) {
    int budget = GC_SWEEP_BUDGET;
    while (gc->rebuild_cur && budget-- > 0) {
        Obj *o = gc->rebuild_cur;
        if (AGE_IS_YOUNG(o->age)) {
            gc->stats.young_live_count--;
            gc->stats.old_live_count++;
        } /* OLD0/OLD1/OLD 已属 old：old_live_count 不变 */
        o->age = MICA_AGE_OLD;
        o->mark = GC_BLACK_BIT; /* OLD 存活保持黑（minor 不碰；major 再重置） */
        gc->rebuild_bytes += obj_size(o);
        gc->rebuild_cur = o->next;
    }
    if (gc->rebuild_cur == NULL) {
        /* 完成：边界指针——一切 OLD，young 区仅 rebuild 期新头插（在 rebuild_first_old 前） */
        gc->survival = gc->rebuild_first_old;
        gc->old1 = gc->rebuild_first_old;
        gc->reallyold = gc->rebuild_first_old;
        gc->firstold1 = NULL;
        gc->old_limit = gc->stats.old_live_count > GC_OLD_LIMIT_MIN
                            ? gc->stats.old_live_count * 2
                            : GC_OLD_LIMIT_MIN;
        /* 自适应 major 基数：本次 major 后存活字节；变 old 记账清零。 */
        gc->gc_major_base = gc->rebuild_bytes;
        gc->gc_marked_old = 0;
        gc->minor_bytes_base = gc->alloc_bytes; /* major 后 young 区小，字节触发基数重置 */
        /* GCv2 OPT-2：记录 major 后精确存活字节（用作下次 major 触发的精确阈值）
           同步清零本周期 GCmarked 计数（OPT-8：下个 major 重累计）。 */
        gc->live_bytes_at_major = gc->live_bytes;
        gc->gc_marked_bytes = 0;
        gc->phase = GC_IDLE;
        gc_set_pause_debt(gc); /* D1：增量完成也设 setpause 债务（下一周期等堆增长） */
#ifdef MICA_GC_DEBUG
        gc_check_orphan_gray(gc);
        gc_check_permanent_roots(gc); /* 增量 major 完成同样校验根登记（显式路径已校验） */
#endif
    }
}

/* 增量推进：每次分配前调用，做有界工作片（GC_ULTRA_PLAN D1 债务模型）。
   债务语义：gc_alloc/gc_account_alloc 减 debt（欠债）；本函数推进分片并每片还债
   GC_SLICE_BYTES；欠债仍负则继续下一片（最多 GC_STEP_MAX_SLICES，Lua incstep 等价）。
   D4 重入防护：step_depth>0（finalizer/回调内分配）直接返回——收集在 IDLE 分支照常运行
   （gc_collect_minor 只查 collect_depth/phase，finalizer 内分配经 gc_alloc → gc_step 已被
   step_depth 挡，不会经此路径重入收集）。 */
void gc_step(Gc *gc) {
    if (MICA_UNLIKELY(gc->step_depth > 0)) return;
    gc->step_depth++;
    double t0 = 0;
    bool measure = gc->stats_enabled && gc->phase != GC_IDLE;
    if (measure) t0 = mica_now_us();
    int slices = 0;
    do {
        switch (gc->phase) {
            case GC_IDLE:
                /* 债务触发 minor（保留 young_limit 兜底防纯大对象低频分配场景；
                   字节触发已并入债务——gc_alloc 记账即减 debt）。 */
                if (gc->stats.young_live_count > gc->young_limit || gc->debt <= 0) {
                    gc_collect_minor(gc);
                }
                /* 自适应 major：自上次 major 以来变 OLD1 的字节达上次 major 存活字节的
                   GC_MINORMAJOR% → 启动增量 major（老对象积累驱动，替代固定轮次）。 */
                if (gc->gc_major_base > 0 &&
                    gc->gc_marked_old >= gc->gc_major_base * GC_MINORMAJOR / 100 &&
                    gc->phase == GC_IDLE) {
                    gc_begin_major(gc);
                }
                /* 轮次驱动：每 GC_OLD_INTERVAL 轮启动增量 major（兜底） */
                if (gc->stats.round > 0 && gc->stats.round % GC_OLD_INTERVAL == 0 &&
                    gc->phase == GC_IDLE) {
                    gc_begin_major(gc);
                }
                /* 兜底：老对象过多即便轮次未到也启动 major */
                if (gc->phase == GC_IDLE && gc->stats.old_live_count > gc->old_limit) {
                    gc_begin_major(gc);
                }
                break; /* IDLE：单片即可（触发或无事） */
            case GC_MARKING:
                if (gc->reset_cur) {
                    
                    gc_reset_slice(gc);
                } else if (gc->explicit_roots
                               ? (gc->root_trace_idx < gc->n_roots || gc->root_arr_obj)
                               : (gc->root_trace_cur || gc->root_arr_obj)) {
                    
                    gc_root_trace_slice(gc);
                } else {
                    gc_mark_slice(gc, GC_MARK_FIELD_BUDGET);
                }
                break;
            case GC_SWEEPING:
                gc_sweep_slice(gc);
                break;
            case GC_REBUILD:
                gc_rebuild_slice(gc);
                break;
            default: break;
        }
        slices++;
        if (gc->phase != GC_IDLE) {
            gc->debt += GC_SLICE_BYTES; /* D1：每片还债（与分配记账对称；粒度近似） */
        } else {
            break;
        }
    } while (gc->debt <= 0 && slices < GC_STEP_MAX_SLICES && gc->phase != GC_IDLE);
    gc->step_depth--;
    if (measure && gc->phase != GC_IDLE) {
        record_pause(gc, mica_now_us() - t0);
    }
}

/* 把增量 major 跑到完成（测试/显式调用/紧急收集用）。 */
void gc_collect_major(Gc *gc) {
    if (!gc->enabled || gc->unsafe_depth > 0) return;
    /* D4：finalizer 内直接调收集 API 被挡（分片执行中 step_depth>0 或收集嵌套）。
       gc_begin_major 须在 collect_depth++ 之前调用（其自身校验 collect_depth==0）。 */
    if (MICA_UNLIKELY(gc->step_depth > 0 || gc->collect_depth > 0)) return;
    if (gc->phase == GC_IDLE) {
        gc_begin_major(gc);
    }
    gc->collect_depth++;
    double t0 = gc->stats_enabled ? mica_now_us() : 0;
    while (gc->phase == GC_MARKING) {
        if (gc->reset_cur) gc_reset_slice(gc);
        else if (gc->explicit_roots
                     ? (gc->root_trace_idx < gc->n_roots || gc->root_arr_obj)
                     : (gc->root_trace_cur || gc->root_arr_obj)) gc_root_trace_slice(gc);
        else gc_mark_slice(gc, SIZE_MAX);
    }
    while (gc->phase == GC_SWEEPING) {
        gc_sweep_slice(gc);
    }
    while (gc->phase == GC_REBUILD) {
        gc_rebuild_slice(gc);
    }
    /* D1：setpause 等价（gc_set_pause_debt：Lua lgc.c:1122-1127） */
    gc_set_pause_debt(gc);
#ifdef MICA_GC_DEBUG
    gc_check_orphan_gray(gc);
    gc_check_permanent_roots(gc);
#endif
    gc->collect_depth--;
    if (gc->stats_enabled) {
        double us = mica_now_us() - t0;
        record_pause(gc, us);
        gc->stats.major_total_us += us;
        if (us > gc->stats.major_max_us) gc->stats.major_max_us = us;
        gc->stats.major_n++;
    }
}

/* ---------- 公共 API ---------- */

/* 在**已零初始化**的 Gc 上做字段初始化（mica_runtime 内嵌 Gc 值用；gc_new 包装它）。 */
void gc_init(Gc *gc) {
    assert(gc != NULL);
    gc->enabled = true;
    gc->young_limit = GC_YOUNG_LIMIT;
    gc->old_limit = GC_OLD_LIMIT_MIN;
    gc->currentwhite = 1; /* WHITE0 */
    gc->pause_samples = NULL; /* 惰性分配（gc_set_stats_enabled 启用时） */
    /* D1：初始债务 = GC_YOUNG_BYTES（首个 minor 在分配满 256KB 后触发，与旧固定阈值一致；
       避免 debt=0 导致首个分配即触发空堆 minor）。explicit_roots 默认 false（现状行为），
       由 mica_runtime_new 登记根后置 true。calloc 已零化 step/collect_depth/n_roots/weakrefs_old。 */
    gc->grayable_bits = MICA_BUILTIN_GRAYABLE_BITS; /* 内置类型；自定义类型由模块注册 */
    gc->debt = (ptrdiff_t)GC_YOUNG_BYTES;
    gc->explicit_roots = false;
    gc->promote_list = NULL;
#ifdef MICA_GC_DEBUG
    gc->debug_checks_enabled = true; /* D10：根覆盖校验默认开（calloc 零化是 false） */
#endif
}

Gc *gc_new(void) {
    Gc *gc = calloc(1, sizeof(Gc));
    assert(gc != NULL);
    gc_init(gc);
    return gc;
}

void gc_register_vtable(Gc *gc, ObjType type, ObjVtable v) {
    assert(type < GC_VTABLE_SLOTS);
    gc->vtables[type] = v;
}

void gc_set_enabled(Gc *gc, bool enabled) {
    gc->enabled = enabled;
}

void gc_set_stats_enabled(Gc *gc, bool enabled) {
    gc->stats_enabled = enabled;
    if (!enabled) {
        free(gc->pause_samples); /* 诊断关闭即释放 512KB 缓冲 */
        gc->pause_samples = NULL;
        gc->pause_n = 0;
    } else if (!gc->pause_samples) {
        gc->pause_samples = malloc(GC_PAUSE_SAMPLES_MAX * sizeof(double));
        assert(gc->pause_samples != NULL);
        gc->pause_n = 0;
    }
}

/* 释放 Gc 内部缓冲（permanent/all_obj 链等），**不含** free(gc)（内嵌 Gc 值时调用）。 */
void gc_deinit(Gc *gc) {
    if (!gc) return;
    free(gc->pause_samples);
    gc->pause_samples = NULL;
    /* 内建方法单例缓存数组本体（vm.c lazily 分配）。其中的 permanent ObjBuiltinMethod
       由下面的 permanent 链统一 free——这里只释放数组本身（每 Gc 一份，故随 Gc 释放）。 */
    free(gc->bm_cache);
    gc->bm_cache = NULL;
    /* 两阶段：先 finalize 全部（permanent + all_obj）再释放。context finalize（obj_context_finalize
       关闭 openupval 捕获值）需要 capval 仍存活——capval 新于 context、在 all_obj 链头侧，
       单遍"finalize+free"会先 free capval 再 finalize context → 悬垂。两阶段保证 finalize 时
       所有对象仍分配（permanent 先 finalize：其 free proto 供运行期闭包 finalize 前释放）。 */
    for (Obj *o = gc->permanent; o; o = o->next) {
        if (o->type < GC_VTABLE_SLOTS && gc->vtables[o->type].finalize) {
            gc->vtables[o->type].finalize(o);
        }
    }
    for (Obj *o = gc->all_obj; o; o = o->next) {
        if (o->type < GC_VTABLE_SLOTS && gc->vtables[o->type].finalize) {
            gc->vtables[o->type].finalize(o);
        }
    }
    for (Obj *o = gc->perm_strs; o; o = o->next) {
        if (o->type < GC_VTABLE_SLOTS && gc->vtables[o->type].finalize) {
            gc->vtables[o->type].finalize(o);
        }
    }
    for (Obj *o = gc->permanent; o;) {
        Obj *next = o->next;
        free(o);
        o = next;
    }
    for (Obj *o = gc->perm_strs; o;) {
        Obj *next = o->next;
        free(o);
        o = next;
    }
    for (Obj *o = gc->all_obj; o;) {
        Obj *next = o->next;
        free(o);
        o = next;
    }
}

void gc_free(Gc *gc) {
    if (!gc) return;
    gc_deinit(gc);
    free(gc);
}

GcStats gc_stats(const Gc *gc) {
    GcStats s = gc->stats;
    s.pause_n = gc->pause_n;
    s.pause_cap = GC_PAUSE_SAMPLES_MAX;
    s.pause_samples = gc->pause_samples;
    s.alloc_total_bytes = gc->alloc_bytes;
    return s;
}

size_t gc_count_permanent(const Gc *gc) {
    size_t n = 0;
    for (const Obj *o = gc->permanent; o; o = o->next) n++;
    for (const Obj *o = gc->perm_strs; o; o = o->next) n++; /* 永久字符串独立链 */
    return n;
}

int gc_phase(const Gc *gc) {
    return gc->phase;
}
