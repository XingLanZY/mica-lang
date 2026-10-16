#include "peephole.h"

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* ============================================================
   窥孔优化驱动 + 初始规则。
   正确性约定：
   - DROP 阶段的规则删除指令前须检查 ctx->is_target / ctx->skip_protected。
   - REWRITE 阶段规则只就地改写（不删指令），可创建新跳转目标；
     DROP 阶段在 REWRITE 之后重算保护信息。
   - 删除后驱动重建 code+lines 并重映射跳转偏移；跳转目标恒保留，故无悬垂。
   ============================================================ */

#define PEEP_MAX_PASSES 16

/* 可开关：状态由 mica_runtime 持有（peephole_state_init 初始化），不再是模块级全局。 */




bool peephole_enabled(const PeepholeState *ps) { return ps->enabled; }
void peephole_set_enabled(PeepholeState *ps, bool on) { ps->enabled = on; }
PeepholeStats peephole_stats(const PeepholeState *ps) { return ps->stats; }
void peephole_stats_reset(PeepholeState *ps) { ps->stats = (PeepholeStats){0}; }

bool peephole_register_rule(PeepholeState *ps, const PeepholeRule *rule) {
    if (!rule || !rule->apply || ps->n_rules >= PEEP_MAX_RULES) return false;
    ps->rules[ps->n_rules++] = rule;
    return true;
}

/* ---------- 保护分析 ---------- */

static bool op_is_jump(OpCode op) {
    return op == OP_JMP || op == OP_JMP_IF_FALSE || op == OP_JMP_IF_TRUE ||
           op == OP_FORI || op == OP_FORL;
}

static bool op_is_skip_compare(OpCode op) {
    return op == OP_ISEQ || op == OP_ISNE || op == OP_ISLT || op == OP_ISLE ||
           op == OP_ISLT_LEN ||
           op == OP_ISEQN || op == OP_ISNEN || op == OP_ISEQS || op == OP_ISNES;
}

static void analyze_protection(const Proto *ch, bool *is_target, bool *skip_protected) {
    for (size_t i = 0; i < ch->count; i++) {
        const OpCode op = instr_op(ch->code[i]);
        if (op_is_jump(op)) {
            const long t = (long)i + 1 + instr_sbx(ch->code[i]); /* 有符号：后向跳转 t 可能 < 0 */
            if (t >= 0 && (size_t)t < ch->count) is_target[t] = true;
        }
        if (op_is_skip_compare(op)) {
            if (i + 1 < ch->count) skip_protected[i + 1] = true;
        }
    }
}

/* ---------- 规则 ---------- */

/* R1：死代码删除——终结指令（RET 族 / EXIT）后，或无条件 JMP 后
   （非 ISLT 跳过目标；ISLT 跳过 JMP 后是 then 分支，不可当死代码），
   到下一个跳转目标/保护目标之间的指令删除。 */
static bool op_terminates(OpCode op) {
    return op == OP_EXIT || op == OP_RET0 || op == OP_RET1 || op == OP_RETURN;
}

static bool rule_is_dead_start(Proto *ch, size_t idx, const PeepholeCtx *ctx) {
    if (op_terminates(instr_op(ch->code[idx]))) return true;
    /* 无条件 JMP：跳过它后不可达（除非是 ISLT 的"跳过下一条"目标） */
    return instr_op(ch->code[idx]) == OP_JMP && !ctx->skip_protected[idx];
}

static PeepholeResult rule_dead_after_ret(Proto *ch, size_t idx, const PeepholeCtx *ctx) {
    if (!rule_is_dead_start(ch, idx, ctx)) return (PeepholeResult){0, 0};
    size_t j = idx + 1;
    while (j < ch->count && !ctx->is_target[j] && !ctx->skip_protected[j]) j++;
    if (j == idx + 1) return (PeepholeResult){0, 0}; /* 无死代码 */
    /* 保留 idx（终结指令），删除其后全部（keep 位仅 bit0=1，j≥1 全删） */
    return (PeepholeResult){(int)(j - idx), 1u};
}

/* R2：MOV r,r 自拷贝删除（regalloc 后常见）。 */
static PeepholeResult rule_self_move(Proto *ch, size_t idx, const PeepholeCtx *ctx) {
    const Instr in = ch->code[idx];
    if (instr_op(in) != OP_MOV || instr_a(in) != instr_b(in)) return (PeepholeResult){0, 0};
    if (ctx->is_target[idx] || ctx->skip_protected[idx]) return (PeepholeResult){0, 0};
    return (PeepholeResult){1, 0u};
}

/* R3：跳转到下一条（sbx=0）的无操作跳转删除（JMP_IF_* 到 next 也删）。 */
static PeepholeResult rule_jump_to_next(Proto *ch, size_t idx, const PeepholeCtx *ctx) {
    const OpCode op = instr_op(ch->code[idx]);
    if (op != OP_JMP && op != OP_JMP_IF_FALSE && op != OP_JMP_IF_TRUE) return (PeepholeResult){0, 0};
    if (instr_sbx(ch->code[idx]) != 0) return (PeepholeResult){0, 0}; /* 目标非下一条 */
    if (ctx->is_target[idx] || ctx->skip_protected[idx]) return (PeepholeResult){0, 0};
    return (PeepholeResult){1, 0u};
}

/* R4：跳转链消除（JMP/条件跳转 → JMP → 直接跳到终点）。就地改写。 */
static PeepholeResult rule_jump_thread(Proto *ch, size_t idx, const PeepholeCtx *ctx) {
    (void)ctx;
    const Instr in = ch->code[idx];
    const OpCode op = instr_op(in);
    if (op != OP_JMP && op != OP_JMP_IF_FALSE && op != OP_JMP_IF_TRUE) return (PeepholeResult){0, 0};
    const long t = (long)idx + 1 + instr_sbx(in);
    if (t < 0 || (size_t)t >= ch->count || instr_op(ch->code[t]) != OP_JMP) return (PeepholeResult){0, 0};
    const long t2 = t + 1 + instr_sbx(ch->code[t]);
    if (t2 == t) return (PeepholeResult){0, 0}; /* 目标 JMP 是自环 */
    ch->code[idx] = instr_sbx_set(in, (int)(t2 - (long)idx - 1));
    return (PeepholeResult){1, 1u};
}

/* R5：返回前 MOV 合并——`MOV r,s; RET1 r` → `RET1 s`（删 MOV，RET1 直接用 s）。
   安全前提：r 由 MOV 定义、RET1 紧邻且是函数结尾，r 无其它使用。
   注意保留 RET1 的 c（结果模式，恒 1）——清 0 会变成返回 0 值 → 调用方得 null。 */
static PeepholeResult rule_mov_before_ret(Proto *ch, size_t idx, const PeepholeCtx *ctx) {
    const Instr in = ch->code[idx];
    if (instr_op(in) != OP_MOV) return (PeepholeResult){0, 0};
    if (idx + 1 >= ch->count || instr_op(ch->code[idx + 1]) != OP_RET1) return (PeepholeResult){0, 0};
    const int r = instr_a(in), s = instr_b(in);
    if (r == s) return (PeepholeResult){0, 0}; /* 自 MOV：由 R2 处理 */
    if (instr_a(ch->code[idx + 1]) != r) return (PeepholeResult){0, 0};
    if (ctx->is_target[idx] || ctx->skip_protected[idx]) return (PeepholeResult){0, 0};
    const Instr ret = ch->code[idx + 1];
    ch->code[idx + 1] = instr_encode(OP_RET1, s, 0, instr_c(ret)); /* 保留结果模式 c */
    return (PeepholeResult){2, 2u}; /* 保留 idx+1（改写后），删 idx */
}

/* ---------- 通用助手 ---------- */

/* 格式统一（2026-08-15）：ABC/AD 的 a 位置不同（ABC@bit7-14、AD@bit8-15），
   寄存器字段判定须按 op 选访问器；b/c 是 ABC 字段（AD 读 d 用 bx/sbx）。 */
static int instr_a_any(Instr in, OpCode op) { return IS_AD_OP(op) ? instr_a_ad(in) : instr_a(in); }
static int instr_b_any(Instr in, OpCode op) { (void)op; return instr_b(in); }
static int instr_c_any(Instr in, OpCode op) { (void)op; return instr_c(in); }

/* 保守检查：from 起寄存器 reg 是否出现在任何"寄存器操作数"字段中。
   用 k_instr_desc 的 IARG_REG 判定真实寄存器字段（dest 也计为使用——保守安全）。
   未用字段（RET0/RET1 的 b/c、JMP 的 a 等）不计。 */
static bool reg_in_any_operand(const Proto *ch, size_t from, int reg) {
    for (size_t i = from; i < ch->count; i++) {
        const Instr in = ch->code[i];
        const OpCode op = instr_op(in);
        const InstrDesc *d = &k_instr_desc[op];
        if (d->a == IARG_REG && instr_a_any(in, op) == reg) return true;
        if (d->b == IARG_REG && instr_b_any(in, op) == reg) return true;
        if (d->c == IARG_REG && instr_c_any(in, op) == reg) return true;
    }
    return false;
}

/* reg 在 from 处是否存活（精确：区分读/写）。从 from 起扫描：遇"读 reg"→ 存活；
   遇"写 reg"（值被覆盖）→ 死。语义对齐 regalloc instr_info 的 uses/defs。
   未知 op 保守视为读（存活），防误删——只用于"寄存器是否可安全复用"的判断。 */
static bool reg_live_at(const Proto *ch, size_t from, int reg) {
    for (size_t i = from; i < ch->count; i++) {
        const Instr in = ch->code[i];
        const OpCode op = instr_op(in);
        const int a = IS_AD_OP(op) ? instr_a_ad(in) : instr_a(in);
        const int b = instr_b(in), c = instr_c(in);
        bool reads = false, writes = false;
        switch (op) {
        case OP_MOV: reads = (b == reg); writes = (a == reg); break;
        case OP_CONST: case OP_KNUM: case OP_KSTR: case OP_MOVI:
        case OP_NULL: case OP_TRUE: case OP_FALSE:
        case OP_LOAD_GLOBAL: case OP_GET_CAPVAL: case OP_CLOSURE: case OP_VARARGS:
            writes = (a == reg); break;
        case OP_ADD: case OP_SUB: case OP_MUL: case OP_DIV: case OP_IDIV: case OP_MOD: case OP_POW:
            reads = (b == reg || c == reg); writes = (a == reg); break;
        case OP_EQ: case OP_NE: case OP_LT: case OP_LE: /* 比较族格式4 */
            reads = (instr_b(in) == reg || instr_c(in) == reg); writes = (instr_a(in) == reg); break;
        case OP_ADDVN: case OP_SUBVN: case OP_MULVN: case OP_DIVVN: case OP_IDIVVN: case OP_MODVN:
            reads = (instr_b(in) == reg); writes = (a == reg); break; /* b 源寄存器（格式3），c 常量 */
        case OP_ADDNV: case OP_MULNV:
            reads = (c == reg); writes = (a == reg); break; /* b 常量，c 源寄存器 */
        case OP_NEG: case OP_NOT:
            reads = (b == reg); writes = (a == reg); break;
        case OP_INC: case OP_DEC:
            reads = (a == reg); writes = (a == reg); break;
        case OP_INDEX:
            reads = (b == reg || c == reg); writes = (a == reg); break;
        case OP_INDEX_SET:
            reads = (a == reg || b == reg || c == reg); break;
        case OP_LENGTH:
            reads = (b == reg); writes = (a == reg); break;
        case OP_GET_MEMBER:
            reads = (c == reg); writes = (a == reg); break;
        case OP_SET_MEMBER:
            reads = (a == reg || c == reg); break;
        case OP_RET1: case OP_RETURN:
            reads = (a == reg); break;
        case OP_RET0: case OP_JMP: break;
        case OP_JMP_IF_FALSE: case OP_JMP_IF_TRUE:
            reads = (a == reg); break;
        case OP_ISEQ: case OP_ISNE: case OP_ISLT: case OP_ISLE:
        case OP_ISLT_LEN:
            reads = (instr_a(in) == reg || instr_b(in) == reg); break; /* 比较族格式4 */
        case OP_ISEQN: case OP_ISNEN: case OP_ISEQS: case OP_ISNES:
            reads = (a == reg); break;   /* b=常量下标 */
        case OP_GETI:
            reads = (b == reg); writes = (a == reg); break;   /* c=常量下标 */
        case OP_SETI:
            reads = (a == reg || b == reg); break;
        case OP_ADDI: case OP_SUBI:
            reads = (b == reg); writes = (a == reg); break;   /* c=imm */
        case OP_STORE_GLOBAL: case OP_THROW: case OP_DEFER_PUSH:
            reads = (a == reg); break;
        default:
            if (a == reg || b == reg || c == reg) reads = true;
            break;
        }
        if (reads) return true;
        if (writes) return false;
    }
    return false;
}

/* R7：常量一元折叠——`MOVI r, C; NEG r` → `MOVI r, -C`（删原 MOVI；须 MOVI 可表示）。
   NOT 不折叠：NOT 产生 bool（非 int），改 MOVI 会变类型；且 int 作布尔操作数会运行期报错。 */
static PeepholeResult rule_const_not_neg(Proto *ch, size_t idx, const PeepholeCtx *ctx) {
    const Instr in = ch->code[idx];
    if (instr_op(in) != OP_MOVI) return (PeepholeResult){0, 0};
    if (idx + 1 >= ch->count || instr_op(ch->code[idx + 1]) != OP_NEG) return (PeepholeResult){0, 0};
    const int r = instr_a(in);
    if (instr_a(ch->code[idx + 1]) != r || instr_b(ch->code[idx + 1]) != r) return (PeepholeResult){0, 0};
    /* idx 与 idx+1 都不能是跳转目标：改写 NEG→MOVI 后，跳到 idx+1 的路径原本对运行期
       值做 NEG，现在直接 MOVI 常量覆盖——语义改变（R10/R12 同样检查 idx+1）。 */
    if (ctx->is_target[idx] || ctx->is_target[idx + 1] || ctx->skip_protected[idx]) return (PeepholeResult){0, 0};
    const int64_t C = instr_sbx(in);
    if (C == INT64_MIN) return (PeepholeResult){0, 0};
    const int64_t newc = -C;
    if (newc < MOVI_MIN || newc > MOVI_MAX) return (PeepholeResult){0, 0};
    ch->code[idx + 1] = instr_sbx_set(instr_encode(OP_MOVI, r, 0, 0), (int)newc);
    return (PeepholeResult){2, 2u}; /* 保留改写后的 MOVI，删原 MOVI */
}

/* R8：死写消除——`MOV r, s; MOV r, t`（同一 dest 相邻）→ 删前一条（写被覆盖）。 */
static PeepholeResult rule_dead_store(Proto *ch, size_t idx, const PeepholeCtx *ctx) {
    const Instr in = ch->code[idx];
    if (instr_op(in) != OP_MOV) return (PeepholeResult){0, 0};
    if (idx + 1 >= ch->count || instr_op(ch->code[idx + 1]) != OP_MOV) return (PeepholeResult){0, 0};
    if (instr_a(in) != instr_a(ch->code[idx + 1])) return (PeepholeResult){0, 0}; /* 同一 dest */
    if (ctx->is_target[idx] || ctx->skip_protected[idx]) return (PeepholeResult){0, 0};
    return (PeepholeResult){2, 2u}; /* 保留 idx+1，删 idx（死写） */
}

/* R9：跳到 RET0 → 就地 RET0——`JMP L`（L 是 RET0）→ `RET0`（省一跳；跳过 ISLT 目标则不做）。 */
static PeepholeResult rule_jump_to_ret0(Proto *ch, size_t idx, const PeepholeCtx *ctx) {
    const Instr in = ch->code[idx];
    if (instr_op(in) != OP_JMP) return (PeepholeResult){0, 0};
    if (ctx->is_target[idx] || ctx->skip_protected[idx]) return (PeepholeResult){0, 0};
    const long t = (long)idx + 1 + instr_sbx(in);
    if (t < 0 || (size_t)t >= ch->count || instr_op(ch->code[t]) != OP_RET0) return (PeepholeResult){0, 0};
    ch->code[idx] = instr_encode(OP_RET0, 0, 0, 0);
    return (PeepholeResult){1, 1u};
}

/* R10：MOV 链收缩——`MOV a, b; MOV c, a`（a 仅被第二条使用、后续无引用）→ `MOV c, b`。 */
static PeepholeResult rule_mov_chain(Proto *ch, size_t idx, const PeepholeCtx *ctx) {
    const Instr in = ch->code[idx];
    if (instr_op(in) != OP_MOV) return (PeepholeResult){0, 0};
    if (idx + 1 >= ch->count || instr_op(ch->code[idx + 1]) != OP_MOV) return (PeepholeResult){0, 0};
    const int a = instr_a(in), b = instr_b(in);
    const Instr in2 = ch->code[idx + 1];
    const int c = instr_a(in2);
    if (instr_b(in2) != a) return (PeepholeResult){0, 0}; /* 第二条读 a */
    if (a == b || a == c || b == c) return (PeepholeResult){0, 0}; /* 别名 */
    if (ctx->is_target[idx] || ctx->skip_protected[idx]) return (PeepholeResult){0, 0};
    if (ctx->is_target[idx + 1] || ctx->skip_protected[idx + 1]) return (PeepholeResult){0, 0};
    if (reg_in_any_operand(ch, idx + 2, a)) return (PeepholeResult){0, 0}; /* a 后续有引用 */
    ch->code[idx + 1] = instr_encode(OP_MOV, c, b, 0);
    return (PeepholeResult){2, 2u}; /* 保留改写后的 MOV c,b，删 MOV a,b */
}

/* R12：常量传播——`MOVI r, C; MOV s, r`（r 仅被 MOV 使用、后续无引用）→ `MOVI s, C`。 */
static PeepholeResult rule_const_prop(Proto *ch, size_t idx, const PeepholeCtx *ctx) {
    const Instr in = ch->code[idx];
    if (instr_op(in) != OP_MOVI) return (PeepholeResult){0, 0};
    if (idx + 1 >= ch->count || instr_op(ch->code[idx + 1]) != OP_MOV) return (PeepholeResult){0, 0};
    const int r = instr_a(in);
    const Instr in2 = ch->code[idx + 1];
    const int s = instr_a(in2);
    if (instr_b(in2) != r || r == s) return (PeepholeResult){0, 0};
    if (ctx->is_target[idx] || ctx->skip_protected[idx]) return (PeepholeResult){0, 0};
    if (ctx->is_target[idx + 1] || ctx->skip_protected[idx + 1]) return (PeepholeResult){0, 0};
    if (reg_in_any_operand(ch, idx + 2, r)) return (PeepholeResult){0, 0}; /* r 后续有引用 */
    const int64_t C = instr_sbx(in);
    ch->code[idx + 1] = instr_sbx_set(instr_encode(OP_MOVI, s, 0, 0), (int)C);
    return (PeepholeResult){2, 2u}; /* 保留改写后的 MOVI s,C，删 MOVI r,C */
}

/* P11：数组下标拷贝折叠（P5）——`MOV t, i; INDEX/INDEX_SET d, a[t]`（t 仅被该下标
   使用、后续无引用）→ `INDEX/INDEX_SET d, a[i]`（删 MOV）。t 是"循环变量 → 临时寄存器"
   拷贝（codegen 求值下标表达式产生）；i 与 t 值相等，改写 c 直接读 i 的寄存器等价。 */
static PeepholeResult rule_index_mov_fold(Proto *ch, size_t idx, const PeepholeCtx *ctx) {
    const Instr in = ch->code[idx];
    if (instr_op(in) != OP_MOV) return (PeepholeResult){0, 0};
    if (idx + 1 >= ch->count) return (PeepholeResult){0, 0};
    const OpCode op2 = instr_op(ch->code[idx + 1]);
    if (op2 != OP_INDEX && op2 != OP_INDEX_SET) return (PeepholeResult){0, 0};
    const int t = instr_a(in), i = instr_b(in);
    const Instr in2 = ch->code[idx + 1];
    if (instr_c(in2) != t) return (PeepholeResult){0, 0};     /* 下标是 t */
    if (t == i) return (PeepholeResult){0, 0};
    if (instr_a(in2) == t || instr_b(in2) == t) return (PeepholeResult){0, 0}; /* t 别处被用 */
    if (ctx->is_target[idx] || ctx->skip_protected[idx]) return (PeepholeResult){0, 0};
    if (ctx->is_target[idx + 1] || ctx->skip_protected[idx + 1]) return (PeepholeResult){0, 0};
    if (reg_live_at(ch, idx + 2, t)) return (PeepholeResult){0, 0}; /* t 后续仍存活（被读） */
    ch->code[idx + 1] = instr_encode(op2, instr_a(in2), instr_b(in2), i);
    return (PeepholeResult){2, 2u}; /* 保留改写后的 INDEX，删 MOV */
}

/* ---------- 内置规则表 ---------- */


/* ---------- 重建（删除后压缩 code+lines 并重映射跳转） ---------- */

static void peephole_rebuild(Proto *ch, const bool *keep) {
    const size_t n = ch->count;
    size_t *map = malloc(n * sizeof(size_t));
    assert(map != NULL);
    /* 第一遍：建 old→new 映射（前向跳转目标在本遍即可查，不会读未初始化） */
    size_t k = 0;
    for (size_t i = 0; i < n; i++) map[i] = keep[i] ? k++ : SIZE_MAX;
    /* 第二遍：压缩 code+lines 并重映射跳转偏移 */
    k = 0;
    for (size_t i = 0; i < n; i++) {
        if (!keep[i]) continue;
        if (k != i) {
            ch->code[k] = ch->code[i];
            ch->lines[k] = ch->lines[i];
        }
        const Instr in = ch->code[k];
        if (op_is_jump(instr_op(in))) {
            const long old_t = (long)i + 1 + instr_sbx(in); /* 有符号：后向跳转 */
            if (old_t >= 0 && (size_t)old_t < n && map[old_t] != SIZE_MAX) {
                const size_t new_t = map[old_t];
                ch->code[k] = instr_sbx_set(in, (int)(new_t - k - 1));
            }
        }
        k++;
    }
    ch->count = k;
    free(map);
}

/* ---------- 驱动（一轮：REWRITE → 重算保护 → DROP） ---------- */

/* 返回是否有任何改动（改写或删除）；any_drop 输出本轮是否有删除。 */
/* ---------- 内置规则表 ---------- */

static const PeepholeRule k_builtin_rules[] = {
        { "jump_thread",   PEEP_PHASE_REWRITE, rule_jump_thread },
        { "dead_after_ret",PEEP_PHASE_DROP,    rule_dead_after_ret },
        { "self_move",     PEEP_PHASE_DROP,    rule_self_move },
        { "jump_to_next",  PEEP_PHASE_DROP,    rule_jump_to_next },
        { "mov_before_ret",PEEP_PHASE_DROP,    rule_mov_before_ret },
        { "const_not_neg", PEEP_PHASE_DROP,    rule_const_not_neg },
        { "dead_store",    PEEP_PHASE_DROP,    rule_dead_store },
        { "jump_to_ret0",  PEEP_PHASE_DROP,    rule_jump_to_ret0 },
        { "mov_chain",     PEEP_PHASE_DROP,    rule_mov_chain },
        { "const_prop",    PEEP_PHASE_DROP,    rule_const_prop },
        { "index_mov_fold",PEEP_PHASE_DROP,    rule_index_mov_fold },
    };

/* 状态初始化：内置规则注册进 state（每状态只初始化一次）。 */
void peephole_state_init(PeepholeState *ps, bool enabled) {
    if (!ps) return;
    ps->enabled = enabled;
    ps->stats = (PeepholeStats){0};
    ps->n_rules = 0;
    for (size_t i = 0; i < sizeof(k_builtin_rules) / sizeof(k_builtin_rules[0]); i++)
        peephole_register_rule(ps, &k_builtin_rules[i]);
}

static bool peephole_pass(Proto *ch, PeepholeState *ps, bool *keep, bool *any_drop) {
    bool changed = false;
    *any_drop = false;

    /* 阶段 A：REWRITE 规则（就地改写，不改指令数；可建新跳转目标） */
    for (size_t i = 0; i < ch->count; ) {
        PeepholeResult best = {0, 0};
        for (size_t r = 0; r < ps->n_rules; r++) {
            if (ps->rules[r]->phase != PEEP_PHASE_REWRITE) continue;
            const PeepholeCtx nctx = {NULL, NULL};
            const PeepholeResult res = ps->rules[r]->apply(ch, i, &nctx);
            if (res.span > 0) { best = res; break; }
        }
        if (best.span <= 0) {
            keep[i] = true;
            i++;
            continue;
        }
        changed = true;
        ps->stats.rules_fired++;
        if ((size_t)best.span > ch->count - i) best.span = (int)(ch->count - i);
        for (int j = 0; j < best.span; j++) keep[i + j] = true;
        i += best.span;
    }

    /* 重算保护（REWRITE 可能创建了新目标） */
    bool *is_target = calloc(ch->count ? ch->count : 1, 1);
    bool *skip_protected = calloc(ch->count ? ch->count : 1, 1);
    assert(is_target != NULL && skip_protected != NULL);
    analyze_protection(ch, is_target, skip_protected);
    const PeepholeCtx ctx = {is_target, skip_protected};

    /* 阶段 B：DROP 规则（用最新保护信息） */
    for (size_t i = 0; i < ch->count; ) {
        PeepholeResult best = {0, 0};
        for (size_t r = 0; r < ps->n_rules; r++) {
            if (ps->rules[r]->phase != PEEP_PHASE_DROP) continue;
            const PeepholeResult res = ps->rules[r]->apply(ch, i, &ctx);
            if (res.span > 0) { best = res; break; }
        }
        if (best.span <= 0) {
            keep[i] = true;
            i++;
            continue;
        }
        changed = true;
        ps->stats.rules_fired++;
        if ((size_t)best.span > ch->count - i) best.span = (int)(ch->count - i);
        for (int j = 0; j < best.span; j++) {
            const bool k = (j < 32) && ((best.keep >> j) & 1) != 0;
            keep[i + j] = k;
            if (!k) *any_drop = true;
        }
        i += best.span;
    }

    free(is_target);
    free(skip_protected);
    return changed;
}

int peephole_run(Proto *ch, PeepholeState *ps) {
    if (!ps || !ps->enabled || !ch || ch->count == 0) return 0;
    long removed_total = 0;
    for (int pass = 0; pass < PEEP_MAX_PASSES; pass++) {
        bool *keep = calloc(ch->count ? ch->count : 1, 1);
        assert(keep != NULL);
        bool any_drop = false;
        const bool changed = peephole_pass(ch, ps, keep, &any_drop);
        if (!changed) { free(keep); break; } /* 无改动：不动点 */
        if (any_drop) {
            size_t drops = 0;
            for (size_t i = 0; i < ch->count; i++)
                if (!keep[i]) drops++;
            peephole_rebuild(ch, keep);
            ps->stats.passes++;
            ps->stats.instructions_removed += (long)drops;
            removed_total += (long)drops;
            if (drops == 0) break;
        }
        free(keep);
    }
    return (int)removed_total;
}
