#ifndef MICA_PEEPHOLE_H
#define MICA_PEEPHOLE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "proto.h"

/* 动态规则表容量（PeepholeState 数组尺寸，故须在结构体前定义） */
#define PEEP_MAX_RULES 64

/* ============================================================
   proto 字节码窥孔优化模块（可开关、规则表驱动、模块化）。
   - 作用：在 regalloc 之后对最终字节码做局部模式改写/删除（消除冗余 MOV/跳转/死代码）。
   - 可开关：模块级开关（codegen 无 runtime 引用，用全局标志），
     mica_set_peephole_enabled / MICA_PEEPHOLE=0 环境变量控制。
   - 扩展：实现 PeepholeRule 并用 peephole_register_rule 注册，驱动自动执行。
   - 正确性保障：驱动保护「跳转目标」和「ISLT 族跳过下一条的目标」不被删除；
     删除指令后重建 code+lines 并重映射跳转偏移。proto_validate 可作回归校验。
   ============================================================ */

/* 规则决策：在 idx 处开一个窗口（span 条原指令），改写其中要保留的指令，
   用 keep 位掩码声明窗口内保留/删除（第 j 位 j<span：1=保留，0=删除）。
   span=0 表示未命中（驱动前进 1 条）。窗口上限 32 条（keep 为 32 位）。
   保留的指令中规则可改写任意一条（如跳转链消除就地改 JMP 偏移）。 */
typedef struct {
    int span;
    uint32_t keep;
} PeepholeResult;

/* 保护信息（每轮规则执行前计算）：规则删除指令前须检查。 */
typedef struct PeepholeCtx {
    const bool *is_target;      /* 跳转目标（JMP 与 JMP_IF 族 / FORI / FORL 指向，不可删） */
    const bool *skip_protected; /* ISLT 族"跳过下一条"的保护目标（不可删） */
} PeepholeCtx;

typedef PeepholeResult (*PeepholeRuleFn)(Proto *ch, size_t idx, const PeepholeCtx *ctx);

typedef enum {
    PEEP_PHASE_REWRITE, /* 先跑：就地改写（可创建新跳转目标，不改指令数） */
    PEEP_PHASE_DROP,    /* 后跑：用最新 is_target 判定删除（可删指令） */
} PeepholePhase;

typedef struct PeepholeRule {
    const char *name;
    PeepholePhase phase;
    PeepholeRuleFn apply;
} PeepholeRule;

/* 统计（调试/测试）。 */
typedef struct {
    long rules_fired;         /* 规则命中次数 */
    long instructions_removed; /* 删除的指令总数 */
    long passes;               /* 驱动轮数 */
} PeepholeStats;

/* 窥孔优化器状态（可开关 + 统计 + 动态规则表）。
   存放在 mica_runtime 中按 runtime 隔离 —— 不再是模块级 static 可变全局
   （历史上全局状态曾导致测试互相污染：自定义规则注册后无法隔离）。 */
typedef struct {
    bool enabled;
    PeepholeStats stats;
    const PeepholeRule *rules[PEEP_MAX_RULES];
    size_t n_rules;
} PeepholeState;

/* 初始化状态（enabled=true，统计归零，规则表为内置规则）。
   须在使用前调用一次（mica_runtime_new 负责；测试可自建栈上实例）。 */
void peephole_state_init(PeepholeState *ps, bool enabled);

/* 对 proto 运行全部规则直到不动点（每轮：REWRITE 阶段 → 重算保护 → DROP 阶段 → 重建）。
   返回删除的指令总数。ps->enabled 为 false 时直接返回 0。 */
int peephole_run(Proto *ch, PeepholeState *ps);

/* 可开关。 */
bool peephole_enabled(const PeepholeState *ps);
void peephole_set_enabled(PeepholeState *ps, bool on);

/* 注册自定义规则（追加到动态表；表满返回 false）。规则指针须保持有效。
   仅影响传入的状态实例，不污染其它 runtime。 */
bool peephole_register_rule(PeepholeState *ps, const PeepholeRule *rule);

/* 统计（调试/测试）。 */
PeepholeStats peephole_stats(const PeepholeState *ps);
void peephole_stats_reset(PeepholeState *ps);

#endif /* MICA_PEEPHOLE_H */
