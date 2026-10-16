/* ============================================================================
 * src/regalloc.c
 *
 * 图着色寄存器分配（codegen 后处理）。
 *
 * 概述：codegen 用线性递增的"虚拟寄存器"发射字节码（语句级 restore_regs 复用）。
 * 本模块读取 proto 的指令序列：
 *   1. 构建控制流图（JMP / JMP_IF_*）。
 *   2. 逆向数据流 liveness（def/use 集合，不动点迭代）。
 *   3. 干扰图：live-in 两两互斥 + def 与 live-out 互斥。
 *   4. 连续寄存器组（CALL 实参/结果、ARRAY/MAP/NEW_ERROR_NAMED 元素块）用
 *      union-find 合并成"块"——块内成员必须着色为连续区间。
 *   5. 贪心着色：固定寄存器（slot0 / 参数 / 被捕获局部）预着色为自身编号，
 *      其余按寄存器序贪心取最小可用色，块整体分配连续色。
 *   6. 按新色重写指令寄存器字段，返回新 max_regs。
 *
 * 安全约束（务必保持）：
 *   - 固定寄存器不可移动（调用约定 slot0/参数、开放捕获值指向的局部槽）。
 *   - 连续组必须保持相邻（VM 的 CALL/ARRAY/MAP 依赖连续槽）。
 *   - max_regs 返回后由调用方写入 Proto.max_regs，驱动 per-context trace 的帧寄存器范围。
 * ========================================================================== */

#include "regalloc.h"

#include <stdlib.h>
#include <string.h>

#include "obj.h"

/* ---------- 位集（256 位 = PROTO_MAX_REGS） ---------- */

typedef struct {
    uint64_t bits[4];
} BitSet;

#define BS_WORDS 4

static void bs_clear(BitSet *s) { memset(s->bits, 0, sizeof(s->bits)); }
static void bs_set(BitSet *s, int i) { s->bits[i >> 6] |= UINT64_C(1) << (i & 63); }
static bool bs_test(const BitSet *s, int i) { return (s->bits[i >> 6] >> (i & 63)) & 1; }
static void bs_or(BitSet *d, const BitSet *a) {
    for (int i = 0; i < BS_WORDS; i++) d->bits[i] |= a->bits[i];
}
static void bs_sub(BitSet *d, const BitSet *a) {
    for (int i = 0; i < BS_WORDS; i++) d->bits[i] &= ~a->bits[i];
}
static bool bs_eq(const BitSet *a, const BitSet *b) {
    for (int i = 0; i < BS_WORDS; i++) if (a->bits[i] != b->bits[i]) return false;
    return true;
}

/* 迭代 live-in 集合中 > lo 的下一个置位寄存器；返回 -1 无。 */
static int bs_next(const BitSet *s, int lo, int n_regs) {
    for (int i = lo + 1; i < n_regs; i++) if (bs_test(s, i)) return i;
    return -1;
}

/* ---------- 指令 def/use 与连续组 ---------- */

typedef struct {
    BitSet defs;
    BitSet uses;
    int group_lo;   /* 连续寄存器组 [lo..hi]；-1 = 无 */
    int group_hi;
} InstrInfo;

/* 算术/比较二元运算（3 操作数 a/b/c）。显式 case 列表——opcode 数值已重排，
   禁数值区间判断，区间在重排后会错位。 */
static bool is_binop(OpCode op) {
    switch (op) {
        case OP_ADD: case OP_SUB: case OP_MUL: case OP_DIV: case OP_IDIV:
        case OP_MOD: case OP_POW: case OP_EQ: case OP_NE: case OP_LT:
        case OP_LE:
            return true;
        default: return false;
    }
}

static int max3(int x, int y, int z) { return x > y ? (x > z ? x : z) : (y > z ? y : z); }

/* 指令引用的最大寄存器号（含连续组末位；varargs 动态段不计，以 a 为界）。
   用于确定 n_regs——不能直接信 codegen 的 max_regs（可能比实际最高寄存器小，
   如 `return 7, 8` 用 R1/R2 但 max_regs 只记到 2，漏算 R2）。 */
static int instr_max_register(Instr in) {
    const OpCode op = instr_op(in);
    /* a 随格式：ABC@bit7-14、AD@bit8-15（b/c 是 ABC 字段；AD 读 d 用 bx/sbx）。 */
    const int a = IS_AD_OP(op) ? instr_a_ad(in) : instr_a(in);
    const int b = instr_b(in), c = instr_c(in);
    switch (op) {
        case OP_CONST: case OP_KNUM: case OP_KSTR: case OP_NULL: case OP_TRUE: case OP_FALSE: case OP_MOVI:
        case OP_LOAD_GLOBAL: case OP_GET_CAPVAL: case OP_VARARGS: case OP_CLOSURE:
        case OP_DEFER_PUSH:
            return a;
        case OP_STORE_GLOBAL: case OP_SET_CAPVAL:
            return a;
        case OP_MOV: case OP_NEG: case OP_NOT: case OP_NEW_ERROR: case OP_IS_ERROR:
        case OP_THROW: case OP_MATCH_ERROR: case OP_GET_ERROR_PAYLOAD: case OP_YIELD:
        case OP_MATCH_ENUM: case OP_MATCH_LITERAL: case OP_IS_TYPE: case OP_GET_ENUM_PAYLOAD:
            return max3(a, b, 0);
        case OP_GET_MEMBER: case OP_SET_MEMBER: case OP_GET_METHOD:
        case OP_GLOAD_INDEX: /* b = 全局名下标（非寄存器） */
            return max3(a, c, 0);
        case OP_INDEX: case OP_INDEX_SET:
            return max3(a, b, c);
        case OP_JMP:
            return 0;
        case OP_JMP_IF_FALSE: case OP_JMP_IF_TRUE:
            return a;
        case OP_ARRAY:
            /* b=元素数；c=首元素寄存器（连续组 c..c+b-1）。
               ⚠️ b==0 时 c 仍是编码字段（重写逻辑读 color[c]）——须包含 c 本身，
               否则 c 为帧外幻影（如 NEW_ERROR_NAMED 曾发射 result+1）时 n_regs 漏算 → color[] 越界
               （2026-08-16 ASAN 修复）。 */
            return max3(a, (b > 0) ? c + b - 1 : c, 0);
        case OP_CONCAT:
            return max3(a, b + c, 0); /* 段寄存器 b..b+c 连续 */
        case OP_LENGTH:
            return max3(a, b, 0);
        case OP_ISEQ: case OP_ISNE: case OP_ISLT: case OP_ISLE:
        case OP_ISLT_LEN:
            return max3(a, b, 0);   /* 比较-跳过：用 R[a] 与 R[b]，无结果寄存器 */
        case OP_ISEQN: case OP_ISNEN: case OP_ISEQS: case OP_ISNES:
            return a;   /* 类型特化比较：R[a] 与常量，无结果寄存器 */
        case OP_GETI:
        case OP_SETI:
            return max3(a, b, 0);   /* 常量下标数组访问：R[a] 与 R[b]（c=常量下标） */
        case OP_ADDI: case OP_SUBI:
            return max3(a, b, 0);   /* 立即数算术：R[a] 与 R[b]（c=imm） */
        case OP_ADDVN: case OP_SUBVN: case OP_MULVN: case OP_DIVVN: case OP_IDIVVN: case OP_MODVN:
            return max3(a, instr_b(in), 0);   /* c = 常量下标（格式3：b 8 位寄存器） */
        case OP_ADDNV: case OP_MULNV:
            return max3(a, c, 0);   /* b = 常量下标 */
        case OP_INC: case OP_DEC:
        case OP_RET1:
            return a;
        case OP_RET0:
            return 0;
        case OP_FORI: case OP_FORL:
            return a + 3;   /* 控制寄存器组 a..a+3 连续 */
        case OP_MAP:
            /* 键值对连续组 c..c+2b-1；b==0 时仍包含 c（见 OP_ARRAY 注释） */
            return max3(a, (b > 0) ? c + 2 * b - 1 : c, 0);
        case OP_ERROR: case OP_ERROR_INFO:
            return 0;
        case OP_CALL:
        case OP_FASTCALL:
        case OP_TAILCALL: {
            const int mode = c & RESULT_MASK;
            int hi = a + b;   /* func + 实参 */
            if (mode != RESULT_DISCARD && mode != RESULT_VARARGS && a + mode - 1 > hi) {
                hi = a + mode - 1;
            }
            return hi;
        }
        case OP_RETURN: {
            const int mode = c & RESULT_MASK;
            return (mode != RESULT_DISCARD && mode != RESULT_VARARGS) ? a + mode - 1 : a;
        }
        case OP_NEW_ERROR_NAMED:
        case OP_NEW_ENUM_NAMED:
            /* 载荷连续组 c..c+b-1；b==0 时仍包含 c（见 OP_ARRAY 注释） */
            return max3(a, (b > 0) ? c + b - 1 : c, 0);
        case OP_DEFER_MARK: case OP_DEFER_RUN:
            return 0;
        default:
            return max3(a, b, c);
    }
}

static void instr_info(const Instr *code, size_t count, const Value *constants, size_t n_constants,
                       int n_regs, size_t i, InstrInfo *info) {
    const Instr in = code[i];
    const OpCode op = instr_op(in);
    const int a = IS_AD_OP(op) ? instr_a_ad(in) : instr_a(in);
    const int b = instr_b(in), c = instr_c(in);
    bs_clear(&info->defs);
    bs_clear(&info->uses);
    info->group_lo = info->group_hi = -1;

    if (is_binop(op)) {
        bs_set(&info->defs, a);
        bs_set(&info->uses, b);
        bs_set(&info->uses, c);
        return;
    }

    switch (op) {
        case OP_CONST: case OP_KNUM: case OP_KSTR: case OP_NULL: case OP_TRUE: case OP_FALSE: case OP_MOVI:
            bs_set(&info->defs, a);
            break;
        case OP_MOV: bs_set(&info->defs, a); bs_set(&info->uses, b); break;
        case OP_LOAD_GLOBAL: bs_set(&info->defs, a); break;
        case OP_STORE_GLOBAL: bs_set(&info->uses, a); break;
        case OP_GET_MEMBER: bs_set(&info->defs, a); bs_set(&info->uses, c); break;
        case OP_SET_MEMBER: bs_set(&info->uses, a); bs_set(&info->uses, c); break;
        case OP_INDEX: bs_set(&info->defs, a); bs_set(&info->uses, b); bs_set(&info->uses, c); break;
        case OP_INDEX_SET: bs_set(&info->uses, a); bs_set(&info->uses, b); bs_set(&info->uses, c); break;
        case OP_GLOAD_INDEX: bs_set(&info->defs, a); bs_set(&info->uses, c); break; /* b=全局名 */
        case OP_NEG: case OP_NOT: bs_set(&info->defs, a); bs_set(&info->uses, b); break;
        case OP_ISEQ: case OP_ISNE: case OP_ISLT: case OP_ISLE:
        case OP_ISLT_LEN:
            bs_set(&info->uses, a); bs_set(&info->uses, b); break;   /* 无 def */
        case OP_ISEQN: case OP_ISNEN: case OP_ISEQS: case OP_ISNES:
            bs_set(&info->uses, a); break;   /* R[a] 与常量，无 def */
        case OP_GETI: bs_set(&info->defs, a); bs_set(&info->uses, b); break;   /* c=常量下标 */
        case OP_SETI: bs_set(&info->uses, a); bs_set(&info->uses, b); break;
        case OP_ADDI: case OP_SUBI: bs_set(&info->defs, a); bs_set(&info->uses, b); break;   /* c=imm */
        case OP_JMP: break;
        case OP_JMP_IF_FALSE: case OP_JMP_IF_TRUE: bs_set(&info->uses, a); break;
        case OP_ARRAY:
            bs_set(&info->defs, a);
            for (int j = 0; j < b; j++) bs_set(&info->uses, c + j);
            if (b > 0) { info->group_lo = c; info->group_hi = c + b - 1; }
            break;
        case OP_CONCAT:
            bs_set(&info->defs, a);
            for (int j = 0; j <= c; j++) bs_set(&info->uses, b + j);
            if (c > 0) { info->group_lo = b; info->group_hi = b + c; }
            break;
        case OP_LENGTH:
            bs_set(&info->defs, a); bs_set(&info->uses, b);
            break;
        case OP_ADDVN: case OP_SUBVN: case OP_MULVN: case OP_DIVVN: case OP_IDIVVN: case OP_MODVN:
            bs_set(&info->defs, a); bs_set(&info->uses, instr_b(in)); break;   /* c = 常量（格式3） */
        case OP_ADDNV: case OP_MULNV:
            bs_set(&info->defs, a); bs_set(&info->uses, c); break;   /* b = 常量 */
        case OP_INC: case OP_DEC:
            bs_set(&info->defs, a); bs_set(&info->uses, a); break;
        case OP_RET0: break;                 /* 无操作数 */
        case OP_RET1: bs_set(&info->uses, a); break;
        case OP_FORI:
            bs_set(&info->uses, a); bs_set(&info->uses, a + 1); bs_set(&info->uses, a + 2);
            bs_set(&info->defs, a + 3);
            info->group_lo = a; info->group_hi = a + 3;
            break;
        case OP_FORL:
            bs_set(&info->uses, a); bs_set(&info->uses, a + 1); bs_set(&info->uses, a + 2);
            bs_set(&info->defs, a + 2); bs_set(&info->defs, a + 3);
            info->group_lo = a; info->group_hi = a + 3;
            break;
        case OP_MAP:
            bs_set(&info->defs, a);
            for (int j = 0; j < 2 * b; j++) bs_set(&info->uses, c + j);
            if (b > 0) { info->group_lo = c; info->group_hi = c + 2 * b - 1; }
            break;
        case OP_ERROR: break;   /* a = 常量下标 */
        case OP_GET_CAPVAL: bs_set(&info->defs, a); break;
        case OP_SET_CAPVAL: bs_set(&info->uses, a); break;
        case OP_CLOSURE: bs_set(&info->defs, a); break;
        case OP_VARARGS: bs_set(&info->defs, a); break;
        case OP_CALL:
        case OP_FASTCALL:
        case OP_TAILCALL: {
            for (int j = 0; j <= b; j++) bs_set(&info->uses, a + j);
            const int mode = c & RESULT_MASK;
            if (mode == RESULT_DISCARD) {
                info->group_lo = a; info->group_hi = a + b;               /* 实参连续 */
            } else if (mode == RESULT_VARARGS) {
                bs_set(&info->defs, a);
                info->group_lo = a; info->group_hi = n_regs - 1;           /* 结果动态，整段连续 */
            } else {
                for (int j = 0; j < mode; j++) bs_set(&info->defs, a + j);
                /* 实参 a+1..a+b 与 结果 a..a+mode-1 的并集必须连续：a..a+max(b, mode-1) */
                const int hi = (b > mode - 1 ? b : mode - 1);
                info->group_lo = a; info->group_hi = a + hi;
            }
            break;
        }
        case OP_RETURN: {
            const int mode = c & RESULT_MASK;
            if (mode != RESULT_DISCARD && mode != RESULT_VARARGS) {
                for (int j = 0; j < mode; j++) bs_set(&info->uses, a + j);
                if (mode > 0) { info->group_lo = a; info->group_hi = a + mode - 1; }
            } else if (mode == RESULT_VARARGS) {
                for (int j = a; j < n_regs; j++) bs_set(&info->uses, j);   /* 动态，保守全用 */
                if (n_regs > a) { info->group_lo = a; info->group_hi = n_regs - 1; }
            }
            break;
        }
        case OP_NEW_ERROR: bs_set(&info->defs, a); bs_set(&info->uses, b); break;
        case OP_NEW_ERROR_NAMED:
        case OP_NEW_ENUM_NAMED:
            bs_set(&info->defs, a);
            for (int j = 0; j < b; j++) bs_set(&info->uses, c + j);
            if (b > 0) { info->group_lo = c; info->group_hi = c + b - 1; }
            break;
        case OP_ERROR_INFO: break;   /* 数据：a/b 是常量下标 */
        case OP_ENUM_INFO: break;    /* 数据：a/b 是常量下标 */
        case OP_IS_ERROR: bs_set(&info->defs, a); bs_set(&info->uses, b); break;
        case OP_THROW: bs_set(&info->uses, a); break;
        case OP_MATCH_ERROR:
        case OP_MATCH_ENUM:
            bs_set(&info->defs, a); bs_set(&info->uses, b); break;
        case OP_MATCH_LITERAL: bs_set(&info->defs, a); bs_set(&info->uses, b); break; /* c=常量 */
        case OP_IS_TYPE: bs_set(&info->defs, a); bs_set(&info->uses, b); break;        /* c=常量 */
        case OP_GET_ERROR_PAYLOAD:
        case OP_GET_ENUM_PAYLOAD:
            bs_set(&info->defs, a); bs_set(&info->uses, b); break; /* c=下标 */
        case OP_GET_METHOD: bs_set(&info->defs, a); bs_set(&info->uses, c); break;   /* b=常量 */
        case OP_YIELD: bs_set(&info->defs, a); bs_set(&info->uses, b); break;
        case OP_DEFER_MARK: case OP_DEFER_RUN: break;
        case OP_DEFER_PUSH: bs_set(&info->uses, a); break;
        default: break;
    }
    (void)constants;
    (void)n_constants;
    (void)count;
}

/* ---------- 跳转目标 ---------- */

static size_t jump_target(size_t i, Instr in) {
    const long t = (long)i + 1 + instr_sbx(in);
    return t >= 0 ? (size_t)t : 0;
}

/* ---------- 干扰图（邻接位集） ---------- */

typedef struct {
    BitSet *adj;   /* adj[r] = 与 r 冲突的寄存器集 */
} Graph;

static void graph_add_edge(Graph *g, int a, int b) {
    if (a == b) return;
    bs_set(&g->adj[a], b);
    bs_set(&g->adj[b], a);
}

/* ---------- union-find（连续组块） ---------- */

static int uf_find(int *uf, int x) {
    while (uf[x] != x) { uf[x] = uf[uf[x]]; x = uf[x]; }
    return x;
}

static void uf_union(int *uf, int a, int b) {
    const int ra = uf_find(uf, a), rb = uf_find(uf, b);
    if (ra != rb) uf[rb] = ra;
}

/* ---------- 主入口 ---------- */

int regalloc_run(Proto *proto, uint32_t arity, uint32_t max_regs) {
    const size_t count = proto->count;
    if (count == 0) return (int)max_regs;

    /* n_regs = max(codegen 的 max_regs, 指令实际引用的最高寄存器 + 1) */
    int n_regs = (int)max_regs;
    for (size_t i = 0; i < count; i++) {
        const int m = instr_max_register(proto->code[i]);
        if (m + 1 > n_regs) n_regs = m + 1;
    }
    if (n_regs <= 0 || n_regs > PROTO_MAX_REGS) return (int)max_regs;

    /* 1. 逐指令 def/use/组 */
    InstrInfo *infos = calloc(count, sizeof(InstrInfo));
    if (!infos) return (int)max_regs;
    for (size_t i = 0; i < count; i++) {
        instr_info(proto->code, count, proto->constants, proto->n_constants, n_regs, i, &infos[i]);
    }

    /* 2. liveness：逆向不动点 */
    BitSet *live_in = calloc(count, sizeof(BitSet));
    BitSet *live_out = calloc(count, sizeof(BitSet));
    if (!live_in || !live_out) {
        free(infos); free(live_in); free(live_out); return (int)max_regs;
    }
    bool changed = true;
    while (changed) {
        changed = false;
        for (size_t i = count; i-- > 0;) {
            const Instr in = proto->code[i];
            const OpCode op = instr_op(in);
            BitSet out;
            bs_clear(&out);
            if (op == OP_JMP) {
                bs_or(&out, &live_in[jump_target(i, in)]);
            } else if (op == OP_JMP_IF_FALSE || op == OP_JMP_IF_TRUE ||
                       op == OP_FORI || op == OP_FORL) {
                /* FORI/FORL 跳循环体（body）：body 存活的跨迭代变量必须传播到循环头，
                   否则会被 body 临时覆盖（如 x 在 `x/guess` 后复用寄存器被覆盖 → 下次迭代读错）。 */
                if (i + 1 < count) bs_or(&out, &live_in[i + 1]);
                bs_or(&out, &live_in[jump_target(i, in)]);
            } else if (op == OP_ISEQ || op == OP_ISNE || op == OP_ISLT || op == OP_ISLE ||
                       op == OP_ISLT_LEN || op == OP_ISEQN || op == OP_ISNEN ||
                       op == OP_ISEQS || op == OP_ISNES) {
                /* 比较-跳过：真 → 跳过下一条（JMP）→ i+2（then/body）；假 → 执行 JMP → 目标。
                   then/body 块存活的变量须传播到比较指令，否则被比较临时覆盖（与 FORL 同因）。
                   类型特化比较（ISEQS/ISNES/ISEQN/ISNEN）同为"真→跳过下一条"，必须一并处理——
                   漏掉则 if/elif 链分支体的活跃变量无法经真分支向后传播 → 与合并点后存活变量
                   漏建干涉边 → 被着色到同一物理槽 → 覆盖活跃参数（2026-08-15 回归：check_achievements）。 */
                if (i + 1 < count) bs_or(&out, &live_in[i + 1]);
                if (i + 2 < count) bs_or(&out, &live_in[i + 2]);
            } else if (i + 1 < count) {
                bs_or(&out, &live_in[i + 1]);
            }
            live_out[i] = out;
            /* in = uses ∪ (out - defs)：uses 必须 live-in（指令读取），
               即使该寄存器同时被本指令定义（如 `R3 = R3 + R1`）。 */
            BitSet in_set = out;
            bs_sub(&in_set, &infos[i].defs);
            bs_or(&in_set, &infos[i].uses);
            if (!bs_eq(&in_set, &live_in[i])) {
                live_in[i] = in_set;
                changed = true;
            }
        }
    }

    /* 3. 干扰图 */
    Graph g;
    g.adj = calloc((size_t)n_regs, sizeof(BitSet));
    if (!g.adj) {
        free(infos); free(live_in); free(live_out); return (int)max_regs;
    }
    for (size_t i = 0; i < count; i++) {
        /* live-in 两两互斥 */
        const BitSet *li = &live_in[i];
        for (int r = bs_next(li, -1, n_regs); r >= 0; r = bs_next(li, r, n_regs)) {
            for (int s = bs_next(li, r, n_regs); s >= 0; s = bs_next(li, s, n_regs)) {
                graph_add_edge(&g, r, s);
            }
        }
        /* def vs live-out */
        const BitSet *ld = &infos[i].defs;
        for (int d = bs_next(ld, -1, n_regs); d >= 0; d = bs_next(ld, d, n_regs)) {
            const BitSet *lo = &live_out[i];
            for (int r = bs_next(lo, -1, n_regs); r >= 0; r = bs_next(lo, r, n_regs)) {
                if (r != d) graph_add_edge(&g, d, r);
            }
            /* def vs live-in：本指令读取的寄存器（如 ARRAY 的元素槽、LE 的操作数）
               不得与写入寄存器共享（VM 先写 R[a] 再读 R[c]，同色会读到新值）。 */
            const BitSet *lvin = &live_in[i];
            for (int r = bs_next(lvin, -1, n_regs); r >= 0; r = bs_next(lvin, r, n_regs)) {
                if (r != d) graph_add_edge(&g, d, r);
            }
        }
    }

    /* 4. 固定寄存器：slot0 + 参数 + 被捕获局部 */
    bool *fixed = calloc((size_t)n_regs, 1);
    if (!fixed) {
        free(infos); free(live_in); free(live_out); free(g.adj); return (int)max_regs;
    }
    fixed[0] = true;
    for (uint32_t i = 1; i <= arity && (int)i < n_regs; i++) fixed[i] = true;
    bool *captured = calloc((size_t)n_regs, 1);
    if (!captured) {
        free(infos); free(live_in); free(live_out); free(g.adj); free(captured); free(fixed); return (int)max_regs;
    }
    for (size_t i = 0; i < count; i++) {
        const Instr in = proto->code[i];
        const OpCode op = instr_op(in);
        if (op != OP_CLOSURE) continue;
        const int k = instr_b(in);
        if (k < 0 || k >= (int)proto->n_constants) continue;
        const Value fv = proto->constants[k];
        if (!val_is_obj(fv) || val_as_obj(fv)->type != OBJ_CLOSURE) continue;
        const ObjClosure *child = (const ObjClosure *)val_as_obj(fv);
        if (!child->proto) continue;
        for (size_t u = 0; u < child->proto->n_capvals; u++) {
            const CapvalDesc *d = &child->proto->capvals[u];
            if (d->is_local && (int)d->index < n_regs) {
                fixed[d->index] = true;
                captured[d->index] = true;
            }
        }
    }
    /* 被捕获槽被开放捕获值隐式持有到帧返回，liveness 看不到——它必须独占寄存器：
       与所有其它寄存器加干扰边，防止任何临时覆盖捕获值（否则 capval 读到错误值）。 */
    for (int c = 0; c < n_regs; c++) {
        if (!captured[c]) continue;
        for (int r = 0; r < n_regs; r++) {
            if (r != c) graph_add_edge(&g, c, r);
        }
    }

    /* 4b. CALL clobber 约束：callee 帧压在 base+func_reg 之上，覆盖该寄存器以上的
       调用者寄存器。codegen 保证"跨调用存活寄存器 < func_reg"（原寄存器空间），
       着色必须保持此顺序：跨调用存活寄存器（live_out - defs）的色 < call 组 func 的色。
       收集 above[r] = 以 r 为 func 的所有调用的跨存活寄存器集（在着色时逐色检查）。 */
    BitSet *above = calloc((size_t)n_regs, sizeof(BitSet));
    if (!above) {
        free(infos); free(live_in); free(live_out); free(g.adj); free(captured); free(fixed);
        return (int)max_regs;
    }
    for (size_t i = 0; i < count; i++) {
        const Instr in = proto->code[i];
        const OpCode op = instr_op(in);
        if (op != OP_CALL && op != OP_FASTCALL) continue;
        const int a = instr_a(in);
        if (a < 0 || a >= n_regs) continue;
        BitSet across = live_out[i];
        bs_sub(&across, &infos[i].defs);   /* 去掉结果寄存器，剩下的都是跨调用存活 */
        bs_or(&above[a], &across);
    }

    /* 5. 连续组 union-find */
    int *uf = malloc((size_t)n_regs * sizeof(int));
    if (!uf) {
        free(infos); free(live_in); free(live_out); free(g.adj); free(captured); free(fixed);
        free(above); return (int)max_regs;
    }
    for (int i = 0; i < n_regs; i++) uf[i] = i;
    for (size_t i = 0; i < count; i++) {
        if (infos[i].group_lo >= 0 && infos[i].group_hi >= infos[i].group_lo &&
            infos[i].group_hi < n_regs) {
            for (int r = infos[i].group_lo; r <= infos[i].group_hi; r++) uf_union(uf, r, infos[i].group_lo);
        }
    }

    /* 6. 贪心着色 */
    int *color = malloc((size_t)n_regs * sizeof(int));
    if (!color) {
        free(infos); free(live_in); free(live_out); free(g.adj); free(captured); free(fixed); free(uf);
        free(above); return (int)max_regs;
    }
    for (int i = 0; i < n_regs; i++) color[i] = -1;
    for (int i = 0; i < n_regs; i++) if (fixed[i]) color[i] = i;   /* 预着色 */

    int max_color = 0;
    for (int r = 0; r < n_regs; r++) {
        if (color[r] >= 0) continue;
        const int root = uf_find(uf, r);

        /* 收集块成员（按原寄存器序，保证内部连续序一致） */
        int members[256];
        int nm = 0;
        for (int m = 0; m < n_regs; m++) {
            if (uf_find(uf, m) == root) members[nm++] = m;
        }

        int c = 0;
        bool found = false;
        while (!found) {
            found = true;
            for (int j = 0; j < nm; j++) {
                const int m = members[j];
                if (color[m] >= 0) {            /* 固定成员：强制 c+j */
                    if (color[m] != c + j) { found = false; break; }
                    continue;
                }
                /* 该成员不可与已着色邻居共享 c+j（块内未着色成员将各得不同色） */
                for (int nb = 0; nb < n_regs; nb++) {
                    if (bs_test(&g.adj[m], nb) && color[nb] == c + j) { found = false; break; }
                }
                if (!found) break;
                /* CALL clobber：若 m 是某调用的 func，跨调用存活寄存器必须低于 c+j */
                for (int x = bs_next(&above[m], -1, n_regs); x >= 0; x = bs_next(&above[m], x, n_regs)) {
                    if (color[x] >= c + j) { found = false; break; }
                }
                if (!found) break;
            }
            if (!found) {
                c++;
                if (c + nm > PROTO_MAX_REGS) {
                    free(infos); free(live_in); free(live_out); free(g.adj);
                    free(captured); free(fixed); free(uf); free(color); free(above);
                    return -1;
                }
            }
        }
        for (int j = 0; j < nm; j++) {
            color[members[j]] = c + j;
            if (c + j > max_color) max_color = c + j;
        }
        r = members[nm - 1];   /* 跳过已着色的块成员 */
    }
    for (int i = 0; i < n_regs; i++) if (color[i] > max_color) max_color = color[i];

    const int new_max_regs = max_color + 1;
    if (new_max_regs > PROTO_MAX_REGS) {
        /* 超出位宽：放弃，保持原样 */
        free(infos); free(live_in); free(live_out); free(g.adj); free(captured); free(fixed); free(uf); free(color); free(above);
        return (int)max_regs;
    }
    if (new_max_regs >= (int)max_regs) {
        /* 无改善：不重写 */
        free(infos); free(live_in); free(live_out); free(g.adj); free(captured); free(fixed); free(uf); free(color); free(above);
        return (int)max_regs;
    }

    /* 7. 重写指令寄存器字段 */
    for (size_t i = 0; i < count; i++) {
        Instr in = proto->code[i];
        const OpCode op = instr_op(in);
        /* a 随格式：ABC@bit7-14、AD@bit8-15（b/c 是 ABC 字段；AD 读 d 用 bx/sbx）。 */
        const int a = IS_AD_OP(op) ? instr_a_ad(in) : instr_a(in);
        const int b = instr_b(in), c = instr_c(in);

        if (is_binop(op) || op == OP_MOV || op == OP_INDEX || op == OP_INDEX_SET ||
            op == OP_NEG || op == OP_NOT) {
            proto->code[i] = instr_encode(op, color[a], color[b], color[c]);
        } else {
            switch (op) {
                /* AD 指令：只改 a（寄存器），d 字段（bx/sbx）保持不动 */
                case OP_CONST: case OP_KNUM: case OP_KSTR: case OP_MOVI:
                case OP_LOAD_GLOBAL: case OP_STORE_GLOBAL:
                case OP_ISEQN: case OP_ISNEN: case OP_ISEQS: case OP_ISNES:
                case OP_JMP_IF_FALSE: case OP_JMP_IF_TRUE:
                case OP_FORI: case OP_FORL:
                    proto->code[i] = instr_rewrite_a_ad(in, color[a]);
                    break;
                /* ABC 单寄存器指令：只有 a 是寄存器 */
                case OP_NULL: case OP_TRUE: case OP_FALSE:
                case OP_GET_CAPVAL: case OP_VARARGS: case OP_CLOSURE:
                case OP_DEFER_PUSH:
                case OP_SET_CAPVAL:
                case OP_INC: case OP_DEC: case OP_RET1:
                    proto->code[i] = instr_rewrite_a(in, color[a]);
                    break;
                case OP_GET_MEMBER: case OP_GET_METHOD: case OP_SET_MEMBER:
                case OP_GLOAD_INDEX:
                    /* b=标识符池下标（不变）、c=对象寄存器 */
                    proto->code[i] = instr_rewrite_bc(instr_rewrite_a(in, color[a]),
                                                      b, color[c]);
                    break;
                case OP_ARRAY: case OP_MAP: case OP_NEW_ERROR_NAMED: case OP_NEW_ENUM_NAMED:
                    /* b=个数（不变）、c=首寄存器（连续组） */
                    proto->code[i] = instr_rewrite_bc(instr_rewrite_a(in, color[a]),
                                                      b, color[c]);
                    break;
                case OP_CALL:
                case OP_FASTCALL:
                case OP_TAILCALL:
                    proto->code[i] = instr_encode(op, color[a], b, c);
                    break;
                case OP_RETURN:
                    proto->code[i] = instr_encode(op, color[a], b, c);
                    break;
                case OP_NEW_ERROR: case OP_IS_ERROR: case OP_THROW: case OP_MATCH_ERROR:
                case OP_GET_ERROR_PAYLOAD: case OP_MATCH_ENUM: case OP_MATCH_LITERAL:
                case OP_IS_TYPE: case OP_GET_ENUM_PAYLOAD:
                case OP_CONCAT: case OP_LENGTH:
                case OP_ISEQ: case OP_ISNE: case OP_ISLT: case OP_ISLE:
                case OP_ISLT_LEN:
                    proto->code[i] = instr_encode(op, color[a], color[b], c);
                    break;
                case OP_ADDVN: case OP_SUBVN: case OP_MULVN: case OP_DIVVN: case OP_IDIVVN: case OP_MODVN:
                    /* b=源寄存器、c=常量池下标（不变） */
                    proto->code[i] = instr_encode(op, color[a], color[b], c);
                    break;
                case OP_ADDNV: case OP_MULNV:
                    proto->code[i] = instr_encode(op, color[a], b, color[c]);
                    break;
                case OP_YIELD:
                    proto->code[i] = instr_encode(op, color[a], color[b], color[c]);
                    break;
                case OP_GETI: case OP_SETI:
                    proto->code[i] = instr_encode(op, color[a], color[b], c);   /* c=常量下标 */
                    break;
                case OP_ADDI: case OP_SUBI:
                    proto->code[i] = instr_encode(op, color[a], color[b], c);   /* c=imm */
                    break;
                default:
                    /* JMP / ERROR / ERROR_INFO / DEFER_MARK / DEFER_RUN：寄存器不变 */
                    break;
            }
        }
    }

    free(infos);
    free(live_in);
    free(live_out);
    free(g.adj);
    free(captured);
    free(fixed);
    free(uf);
    free(color);
    free(above);
    return new_max_regs;
}
