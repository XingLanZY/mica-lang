#ifndef MICA_OPCODES_H
#define MICA_OPCODES_H

/* ============================================================
 * src/opcodes.h
 *
 * 字节码指令编码：LuaJIT 式两种格式（2026-08-10 重构，从 proto.h 拆出；
 *   2026-08-15 再改：op 8→7 位、c 8→9 位连续字段）。
 *
 *   ABC 格式：op(7) | a(8) | b(8) | c(9)   —— 寄存器/常量操作数
 *   AD  格式：op(7) | pad(1) | a(8) | d(16) —— d 为 16 位字段（bx 无符号 / sbx 有符号）
 *
 * 物理位布局（32 位定长）：
 *   ABC：bit0-6 op | bit7-14 a | bit15-22 b | bit23-31 c（c 连续 9 位 → instr_c 直接 >>23）
 *   AD ：bit0-6 op | bit7 pad（恒 0）| bit8-15 a | bit16-31 d
 *
 * 关键：**a 的位置随格式不同**（ABC@bit7-14 / AD@bit8-15）——instr_a 读 ABC，
 *   AD 指令须用 instr_a_ad。b/c 是 ABC 字段；d 是 AD 字段（bx/sbx）。
 *
 * AD 指令（d 字段语义）：
 *   JMP / JMP_IF_FALSE / JMP_IF_TRUE / FORI / FORL   d = sBx 跳转偏移（16 位有符号）
 *   MOVI                                          d = sBx 立即数（16 位有符号）
 *   CONST / KNUM / KSTR                            d = bx 常量池下标
 *   LOAD_GLOBAL / STORE_GLOBAL                     d = bx 全局标识符池下标
 *   ISEQN / ISNEN / ISEQS / ISNES                  d = bx 常量池下标
 *
 * 常量/标识符容量（2026-08-15）：
 *   c 字段 9 位 → ADDVN 族/GETI/SETI 常量下标 0..511（原 8 位 255）。
 *   d 字段 16 位 → CONST/KNUM/KSTR/LOAD_GLOBAL 等 0..65535。
 *   全局标识符池独立（PROTO_MAX_GLOBAL_IDENTS=65536），成员标识符池 PROTO_MAX_CONSTS=512。
 * ============================================================ */

#include <stdbool.h>
#include <stdint.h>

/* 本地内联宏：不依赖 value.h 的 MICA_FORCEINLINE（opcodes.h 可能先于 value.h 被包含）。
   编解码函数在主循环热路径，须强制内联。 */
#if defined(_MSC_VER)
#define MICA_OP_INLINE static __forceinline
#elif defined(__GNUC__) || defined(__clang__)
#define MICA_OP_INLINE static inline __attribute__((always_inline))
#else
#define MICA_OP_INLINE static inline
#endif

/* ---------- 指令类型 ---------- */

/* 32 位定长指令。ABC/AD 两格式（2026-08-15：op 7 位、c 9 位连续字段）：
   ABC：op(7)@0-6 | a(8)@7-14 | b(8)@15-22 | c(9)@23-31
   AD ：op(7)@0-6 | pad@7 | a(8)@8-15 | d(16)@16-31
   编解码统一走 instr_* 访问器（移位+掩码）；热路径编译器消掩码（最高字段逻辑右移直读）。 */
typedef uint32_t Instr;

_Static_assert(sizeof(Instr) == 4, "Instr must be 4 bytes");

#define INSTR_A_SHIFT  7   /* ABC 的 a（bit7-14） */
#define INSTR_B_SHIFT  15  /* ABC 的 b（bit15-22） */
#define INSTR_C_SHIFT  23  /* ABC 的 c（bit23-31，9 位） */
#define INSTR_AD_A_SHIFT 8 /* AD 的 a（bit8-15） */
#define INSTR_D_SHIFT  16  /* AD 的 d（bit16-31） */

/* ---------- 操作码（2026-08-10 显式编号；改动数值须重编 galaxy.mlib） ----------
   编号按热度分组前置（热 op 值最小，i-cache 前端集中）：
   组 0 寄存器/常量移动 0-16；组 1 算术 17-35；组 2 比较 36-48；组 3 成员/长度 49-51；
   组 4 控制流 52-56；组 5 调用/返回 57-61；组 6 闭包/捕获 62-65；
   组 7 字符串/对象构造 66-69；组 8 错误 70-77；组 9 枚举 78-83；
   组 10 协程/defer 84-88；组 11 程序结束 89-90。新增指令在 OP_COUNT 前分配下一个号，
   **禁改既有数值**（位移 → 旧 .mlib 错读，须重编 galaxy.mlib）。 */
typedef enum {
    /* ---- 组 0：寄存器/常量移动（最热） ---- */
    OP_MOV = 0,          /* a=dst, b=src               R[a] = R[b] */
    OP_LOAD_GLOBAL = 1,  /* a=dst, bx=名字常量        R[a] = globals[name] */
    OP_STORE_GLOBAL = 2, /* a=src, bx=名字常量        globals[name] = R[a] */
    OP_INDEX = 3,        /* a=dst, b=obj, c=下标寄存器 R[a] = R[b][R[c]] */
    OP_INDEX_SET = 4,    /* a=值, b=obj, c=下标寄存器  R[b][R[c]] = R[a] */
    OP_GLOAD_INDEX = 5,  /* R[a] = globals[b][R[c]]（超级指令 B，实测频次 #2） */
    OP_ADDI = 6,         /* R[a] = R[b] + imm（有符号 8 位，实测频次 #4） */
    OP_SUBI = 7,         /* R[a] = R[b] - imm */
    OP_GETI = 8,         /* R[a] = R[b][常量下标]（ABC，c=常量下标 8 位） */
    OP_SETI = 9,         /* R[b][常量下标] = R[a] */
    OP_CONST = 10,       /* a=dst, bx=常量下标         R[a] = constants[bx] */
    OP_KNUM = 11,        /* a=dst, bx=数值常量下标     R[a] = num_consts[bx] */
    OP_KSTR = 12,        /* a=dst, bx=字符串常量下标   R[a] = str_consts[bx] */
    OP_MOVI = 13,        /* a=dst, sBx=立即数         R[a] = sBx（16 位有符号立即数） */
    OP_NULL = 14,        /* a=dst                      R[a] = null */
    OP_TRUE = 15,        /* a=dst                      R[a] = true */
    OP_FALSE = 16,       /* a=dst                      R[a] = false */

    /* ---- 组 1：算术 ---- */
    OP_ADD = 17, OP_SUB = 18, OP_MUL = 19, OP_DIV = 20, OP_IDIV = 21, OP_MOD = 22, OP_POW = 23,
    OP_NEG = 24, OP_NOT = 25,
    OP_ADDVN = 26, OP_SUBVN = 27, OP_MULVN = 28, OP_DIVVN = 29, OP_IDIVVN = 30, OP_MODVN = 31,  /* ABC：c=常量下标 8 位 */
    OP_ADDNV = 32, OP_MULNV = 33,  /* 常量在 B（8 位），格式 ABC */
    OP_INC = 34, OP_DEC = 35,

    /* ---- 组 2：比较（LuaJIT 思想精简 14→9：删 GT/GE/ISGT/ISGE 反向操作数交换、
       删死指令 ISGE_LEN；EQ/NE/ISEQ/ISNE 保持独立）。全部 ABC 纯寄存器。
       类型特化相等 compare-skip（ISEQN 族）用 AD：a=寄存器, d=常量下标。 ---- */
    OP_EQ = 36, OP_NE = 37, OP_LT = 38, OP_LE = 39,   /* 产生布尔值：R[a] = R[b] op R[c] */
    OP_ISLT = 40, OP_ISLE = 41, OP_ISEQ = 42, OP_ISNE = 43,   /* 比较-跳过：真则跳过下一条 */
    OP_ISLT_LEN = 44,                                /* 长度比较-跳过：R[a] < R[b].length */
    OP_ISEQN = 45, OP_ISNEN = 46, OP_ISEQS = 47, OP_ISNES = 48,  /* R[a] op 常量：真则跳过下一条（AD） */

    /* ---- 组 3：成员访问 / 长度 ---- */
    OP_GET_MEMBER = 49, OP_SET_MEMBER = 50, OP_LENGTH = 51,

    /* ---- 组 4：控制流（跳转均 AD：a=条件寄存器, d=sBx） ---- */
    OP_JMP = 52, OP_JMP_IF_FALSE = 53, OP_JMP_IF_TRUE = 54,
    OP_FORI = 55, OP_FORL = 56,

    /* ---- 组 5：调用 / 返回 ---- */
    OP_CALL = 57, OP_FASTCALL = 58, OP_RETURN = 59, OP_RET0 = 60, OP_RET1 = 61,

    /* ---- 组 6：闭包 / 捕获 ---- */
    OP_CLOSURE = 62, OP_GET_CAPVAL = 63, OP_SET_CAPVAL = 64, OP_VARARGS = 65,

    /* ---- 组 7：字符串 / 对象构造 ---- */
    OP_CONCAT = 66, OP_GET_METHOD = 67, OP_ARRAY = 68, OP_MAP = 69,

    /* ---- 组 8：错误（冷） ---- */
    OP_ERROR = 70, OP_NEW_ERROR = 71, OP_NEW_ERROR_NAMED = 72, OP_ERROR_INFO = 73,
    OP_IS_ERROR = 74, OP_THROW = 75, OP_MATCH_ERROR = 76, OP_GET_ERROR_PAYLOAD = 77,

    /* ---- 组 9：枚举（冷） ---- */
    OP_NEW_ENUM_NAMED = 78, OP_ENUM_INFO = 79, OP_GET_ENUM_PAYLOAD = 80,
    OP_MATCH_ENUM = 81, OP_MATCH_LITERAL = 82, OP_IS_TYPE = 83,

    /* ---- 组 10：协程 / defer（冷） ---- */
    OP_YIELD = 84,
    OP_DEFER_MARK = 85, OP_DEFER_PUSH = 86, OP_DEFER_RUN = 87,

    /* ---- 组 11：程序结束 ---- */
    OP_EXIT = 88,
    OP_TAILCALL = 89,    /* return f(...) 复用当前帧（P1） */

    OP_COUNT = 90,
} OpCode;

/* AD 格式指令判定：d 字段为 16 位（bx/sbx）。显式 case 列表——禁 opcode 数值区间判断
   （数值重排后区间会错位）。 */
#define IS_AD_OP(op) \
    ((op) == OP_JMP || (op) == OP_JMP_IF_FALSE || (op) == OP_JMP_IF_TRUE || \
     (op) == OP_MOVI || (op) == OP_CONST || (op) == OP_KNUM || (op) == OP_KSTR || \
     (op) == OP_LOAD_GLOBAL || (op) == OP_STORE_GLOBAL || \
     (op) == OP_FORI || (op) == OP_FORL || \
     (op) == OP_ISEQN || (op) == OP_ISNEN || (op) == OP_ISEQS || (op) == OP_ISNES)

/* ---------- opcode 元数据（统一 name + 字段语义，供 proto/disasm/校验查表） ----------
   regalloc 的复杂语义（CALL 结果模式、ARRAY/MAP 连续组）仍由 regalloc.c 专用逻辑处理；
   本表提供统一命名与字段语义来源。 */
typedef enum {
    IARG_UNUSED,  /* 字段未用 */
    IARG_REG,     /* 寄存器 */
    IARG_CONST,   /* 常量池下标 */
    IARG_IDENT,   /* 标识符池下标（全局/成员名） */
    IARG_SBX,     /* d 字段 16 位有符号（跳转偏移 / MOVI 立即数） */
    IARG_RESULT,  /* 结果模式（CALL/FASTCALL/RETURN 的 c） */
    IARG_IMM,     /* 无符号立即数（个数/捕获下标/载荷数等） */
} InstrArgKind;

typedef struct {
    const char *name;
    InstrArgKind a, b, c;
} InstrDesc;

extern const InstrDesc k_instr_desc[OP_COUNT];

/* ---------- 指令编解码（VM 主循环每指令调多次，强制内联） ---------- */

MICA_OP_INLINE OpCode instr_op(Instr in) {
    return (OpCode)(in & 0x7fu); /* 7 位操作码（bit0-6） */
}

/* ABC 格式的 a（bit7-14）。调用方须保证 op 是 ABC 格式——AD 指令的 a 在 bit8-15，
   用 instr_a_ad。 */
MICA_OP_INLINE int instr_a(Instr in) {
    return (int)((in >> INSTR_A_SHIFT) & 0xffu);
}

/* AD 格式的 a（bit8-15；bit7 是 pad）。AD 指令（JMP/CONST/KNUM/KSTR/MOVI/LOAD_GLOBAL/
   STORE_GLOBAL/FORI/FORL/ISEQN 族/JMP_IF）读 a 用本访问器。 */
MICA_OP_INLINE int instr_a_ad(Instr in) {
    return (int)((in >> INSTR_AD_A_SHIFT) & 0xffu);
}

/* ABC 格式字段：b@bit15-22、c@bit23-31（9 位连续）。调用方须保证 op 是 ABC 格式
   （AD 指令读 d 用 instr_bx/instr_sbx）。 */
MICA_OP_INLINE int instr_b(Instr in) {
    return (int)((in >> INSTR_B_SHIFT) & 0xffu);
}

MICA_OP_INLINE int instr_c(Instr in) {
    /* c 在最高 9 位（bit23-31）：逻辑右移 23 直接得值（0..511），无需 & mask */
    return (int)((uint32_t)in >> INSTR_C_SHIFT);
}

/* AD 格式：16 位无符号 d（常量/全局标识符池下标）。d@bit16-31 高位，右移直读。 */
MICA_OP_INLINE int instr_bx(Instr in) {
    return (int)((uint32_t)in >> INSTR_D_SHIFT);
}

/* AD 格式：16 位有符号 d（跳转偏移 / MOVI 立即数）。 */
MICA_OP_INLINE int instr_sbx(Instr in) {
    return (int)(int16_t)(uint16_t)(in >> INSTR_D_SHIFT);
}

/* 把 16 位有符号值 s 写入指令的 d 字段（跳转回填 / MOVI 立即数），
   与 instr_sbx 互为逆运算；保留 op 与 a（含 AD 的 pad@bit7）。 */
MICA_OP_INLINE Instr instr_sbx_set(Instr in, int s) {
    return (Instr)((in & 0xffffu) | ((uint32_t)(uint16_t)s << INSTR_D_SHIFT));
}

/* 通用编码：AD 指令的 b 是完整 16 位 d（c 忽略）；ABC 指令的 b 8 位、c 9 位。 */
MICA_OP_INLINE Instr instr_encode(int op, int a, int b, int c) {
    Instr in = (Instr)(uint8_t)op; /* op@bit0-6；AD 的 pad@bit7 恒 0 */
    if (IS_AD_OP(op)) {
        in |= (uint32_t)(uint8_t)a << INSTR_AD_A_SHIFT;
        in |= (uint32_t)(uint16_t)b << INSTR_D_SHIFT;
    } else {
        in |= (uint32_t)(uint8_t)a << INSTR_A_SHIFT;
        in |= (uint32_t)(uint8_t)b << INSTR_B_SHIFT;
        in |= (uint32_t)(c & 0x1ffu) << INSTR_C_SHIFT;
    }
    return in;
}

/* 重写指令的寄存器字段（regalloc 用）：ABC 的 a（bit7-14）。AD 指令用 instr_rewrite_a_ad。 */
MICA_OP_INLINE Instr instr_rewrite_a(Instr in, int a) {
    return (Instr)((in & ~(0xffu << INSTR_A_SHIFT)) |
                   ((uint32_t)(uint8_t)a << INSTR_A_SHIFT));
}

/* 重写 AD 指令的 a（bit8-15；d 字段保持）。 */
MICA_OP_INLINE Instr instr_rewrite_a_ad(Instr in, int a) {
    return (Instr)((in & ~(0xffu << INSTR_AD_A_SHIFT)) |
                   ((uint32_t)(uint8_t)a << INSTR_AD_A_SHIFT));
}

/* 重写 ABC 指令的 b/c 寄存器字段（regalloc 用；常量/立即数字段由调用方回传）。 */
MICA_OP_INLINE Instr instr_rewrite_bc(Instr in, int b, int c) {
    return (Instr)((in & ~(0xffu << INSTR_B_SHIFT | 0x1ffu << INSTR_C_SHIFT)) |
                   ((uint32_t)(uint8_t)b << INSTR_B_SHIFT) |
                   ((uint32_t)(c & 0x1ffu) << INSTR_C_SHIFT));
}

/* MOVI 立即数的可表示范围（16 位有符号）。 */
#define MOVI_MIN (-32768)
#define MOVI_MAX 32767

static inline bool instr_movi_fits(int64_t v) {
    return v >= MOVI_MIN && v <= MOVI_MAX;
}

/* 结果模式（CALL/FASTCALL/RETURN 的 c 低 7 位）：0=丢弃；127=可变；1..126=固定个数。
   OP_CALL 的 c 高位置 RESULT_PROTECT 表示调用点被 try/catch 保护（运行期 throws 检查豁免）。 */
#define RESULT_DISCARD 0
#define RESULT_VARARGS 127
#define RESULT_PROTECT 0x80
#define RESULT_MASK    0x7f

/* 单函数上限（2026-08-15 字段位宽约束）：
   - 寄存器：A/B 各 8 位 → 256（PROTO_MAX_REGS）。
   - 常量：OP_CONST/KNUM/KSTR 用 d 16 位；ADDVN 族/GETI/SETI 用 c 9 位 → 池上限 512。
   - 成员标识符：GET_MEMBER 的 b 8 位 → 池上限 512（指令可寻址 ≤255）。
   - 全局标识符：LOAD_GLOBAL 的 d 16 位 → 池上限 65536。 */
#define PROTO_MAX_REGS 256
#define PROTO_MAX_CONSTS 512
#define PROTO_MAX_GLOBAL_IDENTS 65536

#endif /* MICA_OPCODES_H */
