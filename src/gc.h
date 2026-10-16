#ifndef MICA_GC_H
#define MICA_GC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "obj.h"

/* 分代垃圾回收（非拷贝式 mark-sweep，Lua 5.5 风格 2 代真分代 + 增量）：
   - 两代：young(age0/1) / old(age2+)。各代是 all_obj 链上的**边界指针**（Lua survival/
     old1/reallyold 风格）标记的代边界，无独立物理中年代。
   - 轮次驱动：young 每轮收集、old 每 GC_OLD_INTERVAL 轮增量 major（自适应字节比例触发）。
   - 写屏障（代际 + 三色）：old 写入 young → 提升 child 到 OLD0（old 代过渡态）入 gray；
     标记期黑 target → gray_again 重标、白 child → gray。
   - young 收集只追踪"根 + gray 链（脏对象）"，不遍历全堆；old 由 markold 有界重扫 OLD1。
   - 所有引用型对象带 gclist → 全部入 gray 链（无原子 trace 兜底需求）。
   - 根 = permanent 对象链（globals/module 表/主 context 都是 permanent）；GC_ULTRA_PLAN
     D2 显式根集模式（explicit_roots=true）下只 trace gc_add_root 登记的根（生产默认开启，
     消灭 O(permanent 全链) 每 minor 固定成本；MICA_GC_ROOTS=0 环境变量回退全链 trace）。
   - 债务驱动（D1）：gc_debt 字节记账（分配减、每片 GC 工作还），工作量 ∝ 分配字节。
   新增对象类型不改本文件：经 gc_register_vtable 注册 trace/finalize 回调。 */

typedef struct Gc Gc;
typedef struct ObjTable ObjTable;

/* GC 阶段（增量收集状态机；gc_phase 返回） */
enum {
    GC_IDLE = 0,   /* 无收集进行中（young 原子收集也在此阶段运行） */
    GC_MARKING,    /* 增量标记中（三色；major 起始的全链重置也在此阶段分片进行） */
    GC_SWEEPING,   /* 增量清扫中 */
    GC_REBUILD,    /* major 清扫完成后增量重建代龄（存活对象分片推进 OLD，消除 O(all) 收尾突刺） */
};

/* 颜色位（ObjHeader.mark）：双白翻转 + 三色。
   bit0 = WHITE0(1)、bit1 = WHITE1(2)（currentwhite 二选一）、bit2 = BLACK(4)、灰 = 0。
   每轮收集"标记完成、清扫开始"时翻转 currentwhite；清扫把存活对象重置为当前白、死对象保持旧白。 */
#define GC_WHITEBITS  3
#define GC_BLACK_BIT  4
#define GC_MASKCOLORS 7

/* 渐进代龄（Lua 5.5 风格；ObjHeader.age 独立 uint8，值 0-5）。
   两代：young = NEW/SURV(0,1)；old = OLD0/OLD1/OLD/TOUCHED(2-5)。
   边界指针（all_obj 链上新→旧）：[all_obj,survival)=NEW、[survival,old1)=SURV、
   [old1,reallyold)=OLD0/OLD1（近期晋升，markold 有界重扫）、[reallyold,)=OLD。
   - NEW：本轮创建（每轮清扫后存活推进为 SURV）。
   - SURV：存活过一轮（仍属 young，可指向任意对象）。
   - OLD0：被写屏障/传播提升（还能指向 young），当轮 link_to_gray 传播。
   - OLD1：上一轮 OLD0（还能指向 survival，markold 每轮重扫后变 OLD）。
   - OLD：真 old，minor/markold 不再碰。
   - TOUCHED：哨兵（无实际对象处于此态）——gc_clear_weakrefs 的 collect_age 用它表示"全部检查"。
   每轮 sweep 自动推进（无需存活计数）：NEW→SURV→OLD1→OLD（OLD0 由 barrier 设置）。 */
enum {
    MICA_AGE_NEW = 0,
    MICA_AGE_SURV,
    MICA_AGE_OLD0,
    MICA_AGE_OLD1,
    MICA_AGE_OLD,
    MICA_AGE_TOUCHED,
};

#define AGE_IS_YOUNG(a)  ((a) <= MICA_AGE_SURV)

#define GC_SWEEP_BUDGET 64   /* major 增量清扫每片最大对象数 */

/* GC 触发阈值（gc.c 使用；热检查内联进 gc_alloc/gc_record_write 需在头文件可见）。
   债务模型（GC_ULTRA_PLAN D1/D9）：gc_debt 负值 = 欠债，触发 gc_step；工作量按片折算还债。
   - GC_MINORMUL：minor 后债务 = 总存活近似字节 × 20%（Lua MINORMUL，lgc.h:185）——
     minor 频率正比于堆大小（大堆少收集、小堆快回收）；下限 GC_YOUNG_BYTES 防小堆病态频繁。
   - GC_PAUSE：major 后债务 = 总存活近似字节 × 250%（Lua PAUSE，lgc.h:191）——
     下一周期等到堆再增长约 150% 存活字节。
   - GC_STEP_MAX_SLICES：一次 gc_step 最多推进片数（Lua incstep do-while 等价，lgc.c:1710-1728）。
   - GC_SLICE_BYTES：每片工作量折算还债字节（粒度近似，够平滑即可）。 */
#define GC_YOUNG_LIMIT 2048      /* young 存活对象数超阈值触发自动 minor（债务模型的兜底） */
#define GC_OLD_LIMIT_MIN 512
#define GC_OLD_INTERVAL 100      /* old 每 100 轮启动增量 major（自适应触发启用后仅兜底） */
#define GC_MINORMAJOR 70         /* 变 OLD1 字节占上次 major 存活字节的 % 时启动增量 major */
#define GC_YOUNG_BYTES (256 * 1024)  /* 债务下限：minor/major 后的最小等待字节（= 旧固定阈值） */
#define GC_MINORMUL 20           /* minor 后债务 = 总存活 × 20%（自适应，Lua MINORMUL） */
#define GC_PAUSE 250             /* major 后债务 = 总存活 × 250%（Lua PAUSE） */
#define GC_STEP_MAX_SLICES 4     /* 一次 gc_step 最多推进片数 */
#define GC_SLICE_BYTES 256       /* 每片工作量折算还债字节 */
#define GC_MARK_FIELD_BUDGET 128 /* major 标记每片字段数 */
#define GC_MARK_OBJ_BUDGET 64    /* major 标记每片对象数 */

/* 显式根集（GC_ULTRA_PLAN D2）：permanent 链上"可能引用运行期对象"的对象显式登记为根，
   explicit_roots=true 时 minor/播种只 trace 根集（消灭 O(permanent 全链) 固定成本）。
   - 登记 = gc_add_root（幂等；mica.c 运行时创建处、测试根、宿主自定义永久根）。
   - 不变量（MICA_GC_DEBUG 自动校验）：未登记 permanent 对象不得引用运行期对象。
   - 默认 false（现状：trace 全 permanent 链，零回归风险）；mica_runtime_new 置 true。 */
#define GC_ROOTS_MAX 64

/* GC_ULTRA_PLAN D7：数组 trace 的 SSE2 批量 itype（ANY 数组 4 元素/组；NUM/BOOL 数组
   经 elements-kind 跳过，无需 SIMD）。x86-64 基线 SSE2（MSVC _M_X64 / GCC Clang __SSE2__）。
   非 x64 或显式关 0 → 标量路径。 */
#if !defined(MICA_GC_SIMD)
#if defined(_M_X64) || defined(__x86_64__)
#define MICA_GC_SIMD 1
#else
#define MICA_GC_SIMD 0
#endif
#endif

/* trace 回调通过该 ctx 的 gc_trace_value/gc_trace_obj 枚举子对象。
   promote=true 时（markold 传播）：trace 到的 young 子对象提升到 OLD0 并置灰入 gray 链，
   把跨代引用的 young 后代闭包整体提升，之后该 old 对象不再指向 young。
   MICA_GC_DEBUG 字段：check_old_young=代际不变量校验（trace 到 young 即断言）；
   check_white_child=根覆盖校验（gc_check_permanent_roots 用：非永久子对象是白 = 未被
   任何根标记 = 登记缺口，即断言）；report_source=当前被校验的永久根（仅报告用）。 */
typedef struct GcTraceCtx {
    Gc *gc;
    bool promote;
#ifdef MICA_GC_DEBUG
    bool check_old_young;
    bool check_white_child;
    const Obj *report_source;
#endif
} GcTraceCtx;

typedef void (*ObjTraceFn)(Obj *obj, GcTraceCtx *ctx);
typedef void (*ObjFinalizeFn)(Obj *obj);

typedef struct {
    ObjTraceFn trace;
    ObjFinalizeFn finalize;
} ObjVtable;

/* vtable 槽位数：基础类型 + 预留槽，外部模块可注册自定义 ObjType（> OBJ_TYPE_COUNT） */
#define GC_VTABLE_SLOTS (OBJ_TYPE_COUNT + 32)

typedef struct {
    size_t minor_count;        /* young 收集次数（每轮一次） */
    size_t major_count;
    size_t round;              /* 轮次计数（每次 young 收集 +1） */
    size_t young_live_count;   /* 当前 young(age0/1) 存活数 */
    size_t old_live_count;     /* 当前 old(age2+) 存活数（含 OLD0/OLD1 过渡态；permanent 不计） */
    /* 性能统计（诊断用；gc_set_stats_enabled(true) 后采集）。pause_samples 指向 Gc
       内部环形缓冲（Gc 存活期间有效），含 gc_collect_minor 整体 + gc_step 非 IDLE 工作片
       的耗时（us，未排序）。宿主可排序求分位分析暂停分布/突刺。 */
    size_t pause_n;            /* 有效样本数（≤ pause_cap） */
    size_t pause_cap;          /* 缓冲容量（GC_PAUSE_SAMPLES_MAX） */
    const double *pause_samples;
    size_t alloc_total_bytes;  /* 累计 gc_alloc 分配字节（含容器记账；不区分代） */
    double minor_total_us;     /* gc_collect_minor 整体暂停累计 */
    double minor_max_us;
    size_t minor_n;
    double major_total_us;     /* gc_collect_major 整体暂停累计（显式全量） */
    double major_max_us;
    size_t major_n;
} GcStats;

/* 暂停样本缓冲容量（环形；超限停止记录丢弃最旧不覆盖）。 */
#define GC_PAUSE_SAMPLES_MAX 65536

/* GC 主结构：定义在此头文件（非 gc.c）——写屏障/分配热路径内联（gc_record_write、
   gc_alloc 触发检查）需在 obj.c/table.c/vm.c 等跨 TU 访问 gc->phase。内部结构，非公开 API。 */
struct Gc {
    Obj *permanent;      /* 编译期永久对象链表 = GC 根来源 */
    Obj *perm_strs;      /* 永久字符串专用链：叶子（无引用），脱离 permanent 根链，
                            GC 根扫描不遍历（省 O(永久字符串数) 开销）；仅 gc_deinit 释放 */
    Obj *all_obj;        /* 所有对象链（head，最新）。2 代边界指针标后代边界（新→旧）：
                            [all_obj,survival)=NEW、[survival,old1)=SURV、
                            [old1,reallyold)=OLD0/OLD1（近期晋升，markold 有界重扫）、
                            [reallyold,)=OLD。 */
    Obj *survival;
    Obj *old1;
    Obj *reallyold;
    Obj *firstold1;      /* [old1,reallyold) 中首个 OLD1（markold 扫描起点；无则 NULL） */

    /* 弱引用注册表链表（OBJ_WEAKREF；不被 trace，不强保持 WeakRef 对象）。
       GC_ULTRA_PLAN D6 双链：weakrefs = 目标 young（每 minor 扫）；weakrefs_old = 目标
       old 代（仅 major 扫；条目由 gc_clear_weakrefs 从 young 链迁移）。 */
    Obj *weakrefs;
    Obj *weakrefs_old;

    /* 运行期协程 context 链（OBJ_CONTEXT，is_main=false）。栈值可能被 resume 无屏障
       写入 young → 每轮重扫（仅 age>=1；young 由根传播覆盖）。主 context 是 permanent 不入链。 */
    Obj *contexts;

    /* GC_ULTRA_PLAN D2 显式根集：explicit_roots=true 时 minor/播种只 trace roots[]。
       登记点：mica_runtime_new（globals/module_inits/module_exports）、
       mica_context_new_sized(is_main=true)、测试根、宿主自定义永久根。 */
    Obj *roots[GC_ROOTS_MAX];
    int n_roots;
    bool explicit_roots;

    /* gray 链：待 trace 的灰对象（所有引用型对象经 gclist 链接）。
       gray_again：IDLE 期=代际脏集合（markold 消费）；MARKING 期=增量重标链表。 */
    Obj *gray;
    Obj *gray_again;

    /* promote_list（P0 修复）：IDLE/REBUILD 期写屏障"提升到 OLD0 待传播"的 young 子对象链。
       **不得与 gray 链混用**：过去 IDLE 期的 link_to_gray 会把 mark 置 0（黑→灰），而清扫
       的唯一存活判据是 isblack ⇒ 活对象被误 free，且该节点仍留在 gray 链上，下一轮
       propagate 对已释放内存写 mark/trace（GC_BUG2.md 记录的"未根除"bug）。
       本链的节点**保持原 mark 不变**，只在收集开始时 splice 进 gray 由 promote 传播消费；
       未消费完的部分下一轮继续（对象仍是 young，由正常清扫规则处理，安全）。 */
    Obj *promote_list;

    /* 增量阶段状态 */
    int phase;               /* GC_IDLE / GC_MARKING / GC_SWEEPING / GC_REBUILD */
    int unsafe_depth;        /* >0 = unsafe 构造窗口：gc_alloc 跳过 gc_step，收集 no-op */
    int currentwhite;        /* 双白当前白（1 或 2），每轮收集标记完成时翻转 */
    Obj **sweepgc;           /* major 增量清扫位置（&某 Obj 的 next 字段） */

    /* grayable 类型位掩码（type < 32 走位查表快路径）：内置类型在 gc_init 登记，
       上层模块的自定义类型由各自 init 经 gc_register_obj_type 注册。
       —— GC 核心不硬编码任何上层模块的类型常量（分层：内存管理不得反向依赖标准库）。 */
    uint32_t grayable_bits;

    /* GC_ULTRA_PLAN D1 字节债务（ptrdiff_t 负 = 欠债）：gc_alloc/gc_account_alloc 减、
       gc_step 每片加 GC_SLICE_BYTES 还债；minor/major 完成时按存活字节设正（D9/setpause）。 */
    ptrdiff_t debt;

    /* GC_ULTRA_PLAN GCv2：精确字节记账（OPT-2）——live_bytes = 实时存活字节（不含 permanent）；
       gc_marked_bytes = 本轮已 trace 的字节数（Lua GCmarked 等价）。每对象 obj_size 精确
       累加，替代原 `(old_live_count + young_live_count) * 64` 估算——估算偏差大时
       setpause/minordebt 不准，导致 GC 触发节律抖动（吞吐 + 突刺）。 */
    int64_t live_bytes;
    int64_t gc_marked_bytes;
    int64_t live_bytes_at_major;  /* 上次 major 完成时的 live_bytes（用于 major 触发阈值） */

    /* GC_ULTRA_PLAN D4 重入防护（Lua gcstopem 等价，lgc.c:1620-1621）——两级深度：
       step_depth：gc_step 重入（finalizer/回调内分配 → gc_alloc → gc_step 被挡）；
       collect_depth：gc_collect_minor/major 重入（finalizer 直接调收集 API 被挡）。
       gc_step 的 IDLE 分支照常调收集（gc_collect_minor 只查 collect_depth/phase；
       gc_collect_major/gc_begin_major 查 step_depth 与 collect_depth）。 */
    int step_depth;
    int collect_depth;

    /* GC_ULTRA_PLAN D5 OOM 处理器：gc_alloc malloc 失败 → 紧急 major + 重试 → 仍失败调
       处理器（NULL=断言）。宿主可注册把 OOM 转成 Mica 错误。 */
    void (*oom_handler)(Gc *gc, size_t size);

    /* 内建方法单例缓存（VM 热路径优化；GC 仅持不透明指针，不解释其内容）：
       get_member 每次访问 `a.push`/`m.has`/`s.trim` 都 new 一个 ObjBuiltinMethod（gc_alloc）
       → 热循环里是每次 GC 分配 + GC 压力。ObjBuiltinMethod 只含 mica_native_fn fn（无状态），
       可共享 permanent 单例。缓存**必须每 Gc 一份**（permanent 对象属某个 Gc；全局缓存
       在 Gc 释放后悬空——test_gc 频繁新建 Gc，malloc 还会复用同地址 → UAF）。
       vm.c 首次使用时 lazily malloc 数组挂此字段；gc_deinit 释放数组本体
       （数组内的 permanent 对象由 permanent 链管理，随 gc_deinit 一并释放）。 */
    void *bm_cache;

#ifdef MICA_GC_DEBUG
    bool debug_checks_enabled; /* D10：根覆盖校验开关（负例测试临时抑制；默认 true） */
#endif

    /* 增量 rebuild 状态（major 清扫完成后把存活对象分片推进 OLD；消除 O(all) 收尾突刺）。
       rebuild_cur != NULL 表示 rebuild 进行中。rebuild_first_old = 首个 OLD 存活（重建锚点）。 */
    Obj *rebuild_cur;
    Obj *rebuild_first_old;
    size_t rebuild_bytes;
    /* 增量 reset 状态（major 起始把全链对象分片重置为当前白；消除 O(all) 起始突刺）。
       reset_cur != NULL 表示 reset 进行中（作为 MARKING 子状态）。 */
    Obj *reset_cur;
    /* 分片播种状态（reset 完成后把 permanent 根子对象分片入 gray，替代一次性 trace_roots：
       大永久根数组 O(all) 单片 trace 是 adaptive major 的突刺来源）。root_trace_cur =
       permanent 链游标（非显式根集模式）；root_trace_idx = 根数组下标游标（显式根集模式）；
       root_arr_obj = 中断的大数组/表（跨片继续）。 */
    Obj *root_trace_cur;
    int root_trace_idx;
    Obj *root_arr_obj;

    ObjVtable vtables[GC_VTABLE_SLOTS];
    bool enabled;
    size_t young_limit;
    size_t old_limit;
    /* major 增量标记的字段级分片状态（大容器 trace 可暂停/恢复） */
    int mark_field_budget;
    Obj *mark_interrupt_obj;
    size_t mark_interrupt_slot;
    bool mark_interrupted;
    /* 自适应 major（Lua 字节比例）：上次 major 以来"变 OLD1"字节 / 上次 major 存活字节。 */
    size_t gc_marked_old;
    size_t gc_major_base;
    /* 字节驱动 minor：累计分配字节 / 上次 young 收集基数（大对象更快触发，控峰值内存）。 */
    size_t alloc_bytes;
    size_t minor_bytes_base;
    /* 性能统计（诊断；gc_set_stats_enabled 启用）。pause_samples 惰性 malloc（默认 NULL，
       不占结构体——512KB 内联数组会膨胀 mica_runtime（Gc 内嵌值）的缓存足迹）；仅诊断启用时分配。 */
    bool stats_enabled;
    double *pause_samples;
    size_t pause_n;
    GcStats stats;
};

/* 启用/禁用收集暂停统计（默认禁用；启用后每次收集多一次时钟读取，~50ns/次）。 */
void gc_set_stats_enabled(Gc *gc, bool enabled);

Gc *gc_new(void);
void gc_free(Gc *gc);

/* 内嵌 Gc 值（mica_runtime.gc）用：gc_init 在已零初始化的 Gc 上初始化字段；
   gc_deinit 释放内部缓冲但**不** free(gc)。gc_new = calloc + gc_init；gc_free = gc_deinit + free。 */
void gc_init(Gc *gc);

/* 注册自定义 GC 对象类型是否可入 gray 链（type 必须 < 32）。
   上层模块（如 librefl）在自己的 init 里调用，GC 核心因此无需知道其类型常量。
   未注册的类型视为非 grayable。可重复调用（幂等）。 */
void gc_register_obj_type(Gc *gc, uint8_t type, bool grayable);
void gc_deinit(Gc *gc);

void gc_register_vtable(Gc *gc, ObjType type, ObjVtable v);

/* 分配：运行期对象（young，可能触发收集）/ 编译期永久对象（不收集）。 */
void *gc_alloc(Gc *gc, size_t size, ObjType type);
void *gc_alloc_permanent(Gc *gc, size_t size, ObjType type);

/* 容器动态缓冲记账（array items / map sparse+dense）：gc_alloc 只记对象头（align16），
   大容器 realloc/malloc 的缓冲在此补记账，让大容器更快触发 minor（控峰值内存更准）。
   delta 为净增字节（不含释放；累计语义与 alloc_bytes 一致）。enabled=false 时 no-op。 */
void gc_account_alloc(Gc *gc, size_t delta);

void gc_collect_minor(Gc *gc);
void gc_collect_major(Gc *gc);
/* 增量推进：做有界工作片（每次分配前由 gc_alloc 调用；测试可显式调用驱动增量收集）。 */
void gc_step(Gc *gc);
/* 强制从 IDLE 启动一次增量 major（标记阶段开始；测试/宿主手动触发用）。 */
void gc_begin_major(Gc *gc);
/* 当前阶段：GC_IDLE / GC_MARKING / GC_SWEEPING / GC_REBUILD。 */
int gc_phase(const Gc *gc);
void gc_set_enabled(Gc *gc, bool enabled);   /* false=分配不收集（测试/调试） */

/* 弱引用（WeakRef）：注册到 GC 弱引用链表（不被 trace，不强保持 WeakRef 对象）。
   收集的"标记完成、清扫前"钩子调 gc_clear_weakrefs(collect_age)——本收集清扫的最大代龄
   （young=SURV/major=TOUCHED=哨兵"全部"），把目标已死的 weakref.target 置 null。
   D6：注册进 young 目标链；gc_clear_weakrefs 把老目标条目迁移到 old 链（major-only）。 */
void gc_add_weakref(Gc *gc, ObjWeakRef *r);
void gc_clear_weakrefs(Gc *gc, int collect_age);

/* ---------- GC_ULTRA_PLAN D2：显式根集 ---------- */

/* 登记 permanent 根（幂等；非 permanent 忽略）。explicit_roots=true 时 minor/播种只 trace
   已登记根。遗漏登记 = 该根引用的运行期对象被 minor 误回收——MICA_GC_DEBUG 自动校验。 */
void gc_add_root(Gc *gc, Obj *o);
/* 查询对象是否已登记为根（调试校验用；O(n_roots)，仅冷路径）。 */
bool gc_is_root(const Gc *gc, const Obj *o);
/* 开/关显式根集模式（默认 false = trace 全 permanent 链，现状行为零回归；
   mica_runtime_new 登记根后置 true）。 */
void gc_set_explicit_roots(Gc *gc, bool enabled);

/* ---------- GC_ULTRA_PLAN D5：OOM 处理器 ---------- */

/* 注册 OOM 处理器：gc_alloc malloc 失败 → 紧急 major 重试 → 仍失败调处理器
   （NULL=默认断言）。处理器内不得再分配（会递归）。 */
void gc_set_oom_handler(Gc *gc, void (*handler)(Gc *gc, size_t size));

#ifdef MICA_GC_DEBUG
/* 调试校验开关（仅 MICA_GC_DEBUG 构建）：gc_check_permanent_roots 根覆盖校验的临时抑制。
   测试"故意违反不变量"的负例（如显式根集语义验证）时使用，必须配对恢复。 */
void gc_set_debug_checks(Gc *gc, bool enabled);
#endif

/* vtable trace 回调内调用：对字段值/对象做标记（统一三色）。 */
void gc_trace_value(GcTraceCtx *ctx, const Value *v);
void gc_trace_obj(GcTraceCtx *ctx, Obj **o);

/* 写屏障慢路径（IDLE promote + REBUILD/MARKING/SWEEPING 增量三色）。内联包装
   gc_record_write 挡掉热路径（IDLE + young target / 非 obj 值）后调此函数。 */
void gc_record_write_slow(Gc *gc, Obj *target, Value v);

/* 写屏障（代际 + 增量三色）：
   - IDLE：old target 写入 young child → 提升 child 到 OLD0 入 gray（防 young 被扫）。
   - MARKING：黑 target → gray_again 重标；白 child → gray。
   - SWEEPING：白 child → 黑（新引用存活）。
   热路径（IDLE + young target / 标量写）内联：VM 热写点（array push/set、实例字段、
   全局 store、map set）免函数调用。 */
MICA_FORCEINLINE void gc_record_write(Gc *gc, Obj *target, Value v) {
    if (MICA_LIKELY(gc->phase == GC_IDLE)) {
        if (MICA_LIKELY(AGE_IS_YOUNG(target->age))) return;
        if (MICA_LIKELY(!val_is_obj(v))) return;
    }
    gc_record_write_slow(gc, target, v);
}

/* unsafe 构造窗口：多步对象构造（未入根的临时对象跨多次 gc_alloc）期间禁止收集。
   窗口内 gc_alloc 跳过 gc_step，显式 gc_collect_* 也 no-op。必须配对。 */
void gc_begin_unsafe(Gc *gc);
void gc_end_unsafe(Gc *gc);

/* 显式释放一个 permanent 对象（从永久链摘除 + vtable finalize + free）。
   permanent 对象永不回收，宿主/运行时主动弃用（如重置模块表、释放主 context）时调用，
   否则其内存与内容会残留到 gc_free。 */
void gc_free_permanent(Gc *gc, Obj *o);

/* 把运行期协程 context 挂入 contexts 链（mica_context_new_sized 调用；栈值每轮重扫）。
   主 context 是 permanent，不入链。 */
void gc_add_context(Gc *gc, Obj *o);

/* 把运行期协程 context 从 contexts 链摘除（obj_context_finalize 调用；context 被 GC 回收时）。
   主 context 是 permanent，不入 contexts 链。 */
void gc_remove_context(Gc *gc, Obj *o);

GcStats gc_stats(const Gc *gc);

/* 统计 gc->permanent 永久链表中的对象数（诊断/测试用）。 */
size_t gc_count_permanent(const Gc *gc);

#endif /* MICA_GC_H */
