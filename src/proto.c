#include "proto.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "obj.h"

Proto *proto_new(void) {
    Proto *ch = calloc(1, sizeof(Proto));
    assert(ch != NULL);
    return ch;
}

/* ---------- 合并池（code + 三常量池 + 标识符池 + 全局标识符池 单块内存） ----------
   布局：[code][constants][num_consts][str_consts][ident_names][global_ident_names]，
   六个指针指向同一段六处。常量池按类型分三区（constants 通用 / num_consts 数值 /
   str_consts 字符串），各自独立下标空间（2026-08-10）。标识符池（成员名）与全局标识符池
   （全局变量名）独立配额（2026-08-15）。
   **对齐**：Instr 4 字节、Value/String 8 字节——各区间偏移显式 align8。从后往前搬。 */

static size_t align8(size_t n) {
    return (n + 7) & ~(size_t)7;
}

static void proto_pool_repoint(Proto *ch) {
    char *base = (char *)ch->pool;
    ch->code = (Instr *)base;
    size_t off = align8(ch->cap * sizeof(Instr));
    ch->constants = (Value *)(base + off);
    off = align8(off + ch->cap_constants * sizeof(Value));
    ch->num_consts = (Value *)(base + off);
    off = align8(off + ch->cap_num_consts * sizeof(Value));
    ch->str_consts = (Value *)(base + off);
    off = align8(off + ch->cap_str_consts * sizeof(Value));
    ch->ident_names = (String *)(base + off);
    off = align8(off + ch->cap_ident_names * sizeof(String));
    ch->global_ident_names = (String *)(base + off);
}

static size_t proto_pool_bytes(size_t cap_code, size_t cap_c, size_t cap_num, size_t cap_str,
                               size_t cap_i, size_t cap_gi) {
    size_t off = align8(cap_code * sizeof(Instr));
    off = align8(off + cap_c * sizeof(Value));
    off = align8(off + cap_num * sizeof(Value));
    off = align8(off + cap_str * sizeof(Value));
    off = align8(off + cap_i * sizeof(String));
    return off + cap_gi * sizeof(String);
}

/* 保证池容量 ≥ need_code/need_c/need_num/need_str/need_i/need_gi；不足时整体 realloc 并按最终布局搬移各区。 */
void proto_reserve_pool(Proto *ch, size_t need_code, size_t need_c, size_t need_num,
                        size_t need_str, size_t need_i, size_t need_gi) {
    size_t cap_code = ch->cap ? ch->cap : 64;
    size_t cap_c = ch->cap_constants ? ch->cap_constants : 16;
    size_t cap_num = ch->cap_num_consts ? ch->cap_num_consts : 16;
    size_t cap_str = ch->cap_str_consts ? ch->cap_str_consts : 16;
    size_t cap_i = ch->cap_ident_names ? ch->cap_ident_names : 16;
    size_t cap_gi = ch->cap_global_ident_names ? ch->cap_global_ident_names : 16;
    while (cap_code < need_code) cap_code *= 2;
    while (cap_c < need_c) cap_c *= 2;
    while (cap_num < need_num) cap_num *= 2;
    while (cap_str < need_str) cap_str *= 2;
    while (cap_i < need_i) cap_i *= 2;
    while (cap_gi < need_gi) cap_gi *= 2;
    if (cap_code == ch->cap && cap_c == ch->cap_constants && cap_num == ch->cap_num_consts &&
        cap_str == ch->cap_str_consts && cap_i == ch->cap_ident_names && cap_gi == ch->cap_global_ident_names)
        return;

    const size_t old_code = ch->cap, old_c = ch->cap_constants, old_num = ch->cap_num_consts;
    const size_t old_str = ch->cap_str_consts, old_i = ch->cap_ident_names, old_gi = ch->cap_global_ident_names;
    const size_t old_c_off = align8(old_code * sizeof(Instr));
    const size_t old_num_off = align8(old_c_off + old_c * sizeof(Value));
    const size_t old_str_off = align8(old_num_off + old_num * sizeof(Value));
    const size_t old_id_off = align8(old_str_off + old_str * sizeof(Value));
    const size_t old_gi_off = align8(old_id_off + old_i * sizeof(String));
    void *np = realloc(ch->pool, proto_pool_bytes(cap_code, cap_c, cap_num, cap_str, cap_i, cap_gi));
    assert(np != NULL);
    ch->pool = np;
    char *base = (char *)np;
    const size_t new_c_off = align8(cap_code * sizeof(Instr));
    const size_t new_num_off = align8(new_c_off + cap_c * sizeof(Value));
    const size_t new_str_off = align8(new_num_off + cap_num * sizeof(Value));
    const size_t new_id_off = align8(new_str_off + cap_str * sizeof(Value));
    const size_t new_gi_off = align8(new_id_off + cap_i * sizeof(String));
    /* 从后往前搬（后区先移，避免覆盖前区尚未搬的源）：
       全局标识符 → 标识符 → 字符串 → 数值 → 通用常量 */
    const size_t gi_bytes = old_gi * sizeof(String);
    if (gi_bytes > 0 && new_gi_off != old_gi_off) {
        memmove(base + new_gi_off, base + old_gi_off, gi_bytes);
    }
    const size_t id_bytes = old_i * sizeof(String);
    if (id_bytes > 0 && new_id_off != old_id_off) {
        memmove(base + new_id_off, base + old_id_off, id_bytes);
    }
    const size_t str_bytes = old_str * sizeof(Value);
    if (str_bytes > 0 && new_str_off != old_str_off) {
        memmove(base + new_str_off, base + old_str_off, str_bytes);
    }
    const size_t num_bytes = old_num * sizeof(Value);
    if (num_bytes > 0 && new_num_off != old_num_off) {
        memmove(base + new_num_off, base + old_num_off, num_bytes);
    }
    const size_t const_bytes = old_c * sizeof(Value);
    if (const_bytes > 0 && new_c_off != old_c_off) {
        memmove(base + new_c_off, base + old_c_off, const_bytes);
    }
    if (cap_code != old_code) {
        ch->lines = realloc(ch->lines, cap_code * sizeof(int)); /* lines 与 code 同容量 */
        assert(ch->lines != NULL);
    }
    ch->cap = cap_code;
    ch->cap_constants = cap_c;
    ch->cap_num_consts = cap_num;
    ch->cap_str_consts = cap_str;
    ch->cap_ident_names = cap_i;
    ch->cap_global_ident_names = cap_gi;
    proto_pool_repoint(ch);
}

void proto_free(Proto *ch) {
    if (!ch) return;
    free(ch->lines);
    free(ch->capvals);
    for (size_t i = 0; i < ch->n_ident_names; i++) str_free(&ch->ident_names[i]); /* 长标识符释放 lstr */
    for (size_t i = 0; i < ch->n_global_ident_names; i++) str_free(&ch->global_ident_names[i]);
    free(ch->pool); /* code + constants + ident_names 全在 pool */
    if (ch->global_entries) free(ch->global_entries);
    free(ch->ics); /* 调用点 IC（D） */
    free(ch->ident_htab);        /* 标识符去重哈希索引（编译期） */
    free(ch->global_ident_htab);
    free(ch);
}

/* 索引化全局：解析 proto 的 LOAD_GLOBAL/STORE_GLOBAL ident 下标 → 全局表稠密下标缓存。
   惰性解析一次（vm.c 首访触发）：稠密下标在表 rehash / dense 增长后仍稳定
   （globals 只增不删），无需版本守卫、无需重解析。未定义（-1）的全局由 STORE_GLOBAL
   定义时更新缓存；LOAD_GLOBAL 遇 -1 走惰性重查（定义后缓存生效）。 */
void proto_resolve_globals(const ObjTable *globals, Proto *ch) {
    if (ch->global_entries) return; /* 已解析（下标永久稳定） */
    size_t maxb = 0;
    for (size_t i = 0; i < ch->count; i++) {
        const OpCode op = instr_op(ch->code[i]);
        if (op == OP_LOAD_GLOBAL || op == OP_STORE_GLOBAL || op == OP_GLOAD_INDEX) {
            /* LOAD_GLOBAL/STORE_GLOBAL 是 AD（bx=16 位标识符下标）；GLOAD_INDEX 是 ABC（b=8 位） */
            const size_t b = (op == OP_GLOAD_INDEX) ? instr_b(ch->code[i]) : instr_bx(ch->code[i]);
            if (b > maxb) maxb = b;
        }
    }
    int32_t *ge = calloc(maxb + 1, sizeof(int32_t));
    assert(ge != NULL);
    for (size_t i = 0; i <= maxb; i++) ge[i] = -1;
    for (size_t i = 0; i < ch->count; i++) {
        const OpCode op = instr_op(ch->code[i]);
        if (op == OP_LOAD_GLOBAL || op == OP_STORE_GLOBAL || op == OP_GLOAD_INDEX) {
            /* LOAD_GLOBAL/STORE_GLOBAL 是 AD（bx=16 位全局标识符下标）；GLOAD_INDEX 是 ABC（b=8 位） */
            const size_t b = (op == OP_GLOAD_INDEX) ? instr_b(ch->code[i]) : instr_bx(ch->code[i]);
            if (ge[b] < 0) {
                const String *name = &ch->global_ident_names[b];
                const size_t d = objtable_find_dense(globals, name->hash, str_data(name),
                                                     str_len(name));
                ge[b] = (d == TABLE_NULL_ELEM) ? -1 : (int32_t)d;
            }
        }
    }
    ch->global_entries = ge;
}

/* ---------- 池去重加速：开放寻址哈希 ----------
   历史实现是**每加一个元素就线性扫一遍全池**（proto_add_ident/global_ident/const/num/str
   共 5 处）。这在"同名/同常量反复出现"时无所谓（命中早），但在**大量互不相同**的名字/
   常量下退化成 O(n²)：实测 20000 个不同全局名 415ms，而 20000 个重复名只要 40ms；
   上限是 65536，最坏情况要几十秒。这里给每个池配一个开放寻址哈希索引：
     - 槽存"池下标 + 1"（0 = 空），下标从 1 起避免与 0 混淆；
     - 表长取 ≥ 2×需求 的 2 的幂，用 h % cap 定位（探测走 (h+i) % cap）；
     - 只在扩容/插入时维护；查表失败时用池内线性兜底一次（哈希未命中必须判负）。
   注意：哈希只用于**加速查找**，权威数据仍是池数组本身，所以任何哈希瑕疵最多退化成
   线性查找，不会产生错误结果（除扩容时机）。 */
static size_t pool_hash_cap(size_t need) {
    size_t cap = 16;
    while (cap < need) cap *= 2;
    return cap * 2;
}

/* 标识符池（成员名 / 全局名）共用：按内容查重，命中返回下标，否则插入并返回新下标。
   返回 -1 表示超上限。which: 0=ident_names（成员名），1=global_ident_names（全局名）。 */
static int ident_pool_intern(Proto *ch, const char *name, size_t len, int which) {
#if defined(MICA_IDENT_LINEAR_DEDUP) && MICA_IDENT_LINEAR_DEDUP
    /* A/B 基准用：旧的线性扫描去重（O(n²)），仅用于量化哈希索引的收益。 */
    String *a2 = (which == 0) ? ch->ident_names : ch->global_ident_names;
    uint32_t *n2 = (which == 0) ? &ch->n_ident_names : &ch->n_global_ident_names;
    const size_t lim2 = (which == 0) ? PROTO_MAX_CONSTS : PROTO_MAX_GLOBAL_IDENTS;
    for (size_t i = 0; i < (size_t)*n2; i++) {
        if (str_equals(a2[i], name, len)) return (int)i;
    }
    if ((size_t)*n2 >= lim2) return -1;
    proto_reserve_pool(ch, ch->count, ch->n_constants, ch->n_num_consts, ch->n_str_consts,
                       which == 0 ? *n2 + 1 : ch->n_ident_names,
                       which == 0 ? ch->n_global_ident_names : *n2 + 1);
    a2 = (which == 0) ? ch->ident_names : ch->global_ident_names;
    a2[*n2] = str_new(name, len);
    return (int)(*n2)++;
#else
    /* ⚠️ 哈希必须与池里 String.hash 完全一致：hash 由 str_new 内部对**最终存储的字节**
       计算（name 已被调用方做 @ 归一）。这里用同一算法对同一字节计算，
       查表命中判据同时用 hash 与 str_equals（hash 只做快速拒绝）。 */
    const uint32_t h = fnv_hash(name, len);
    String *arr = (which == 0) ? ch->ident_names : ch->global_ident_names;
    /* 注意类型：池计数是 uint32_t（见 proto.h），**不能**用 size_t* 取地址——
       那会做 64 位读，把相邻字段一起读进来（实测读出 0x100000000）。 */
    uint32_t *pn = (which == 0) ? &ch->n_ident_names : &ch->n_global_ident_names;
    uint32_t **htab = (which == 0) ? &ch->ident_htab : &ch->global_ident_htab;
    size_t *hcap = (which == 0) ? &ch->ident_htab_cap : &ch->global_ident_htab_cap;
    const size_t limit = (which == 0) ? PROTO_MAX_CONSTS : PROTO_MAX_GLOBAL_IDENTS;
    size_t n = (size_t)*pn;

    if (*htab) {
        for (size_t i = 0; i < *hcap; i++) {
            const size_t slot = ((size_t)h + i) % *hcap;
            const uint32_t e = (*htab)[slot];
            if (e == 0) break;                        /* 空槽：确定未命中 */
            const size_t k = (size_t)e - 1;
            if (k < n && arr[k].hash == h && str_equals(arr[k], name, len)) return (int)k;
        }
        /* 表是"只增 + 负载>70% 重建"，且每次插入都写了槽，故"遇空槽"必是真的未命中；
           无需线性兜底（那会把 O(1) 打回 O(n)）。 */
    }

    if (n >= limit) return -1;
    proto_reserve_pool(ch, ch->count, ch->n_constants, ch->n_num_consts, ch->n_str_consts,
                       which == 0 ? n + 1 : ch->n_ident_names,
                       which == 0 ? ch->n_global_ident_names : n + 1);
    arr = (which == 0) ? ch->ident_names : ch->global_ident_names; /* realloc 后重取 */
    arr[n] = str_new(name, len);
    /* 用**存入后读回**的 hash，彻底避免"计算 hash 与存储 hash 不一致"这类隐蔽失配 */
    const uint32_t sh = arr[n].hash;
    assert(sh == h); /* 同算法同输入必须同结果（strlit 常量表的假设也依赖这一点） */

    if (*htab == NULL || (n + 1) * 10 >= *hcap * 7) {  /* 首次 / 负载 >70% 重建 */
        const size_t ncap = pool_hash_cap(n + 1);
        uint32_t *nt = (uint32_t *)calloc(ncap, sizeof(uint32_t));
        assert(nt != NULL);
        for (size_t i = 0; i < n; i++) {
            for (size_t j = 0; j < ncap; j++) {
                const size_t slot = ((size_t)arr[i].hash + j) % ncap;
                if (nt[slot] == 0) { nt[slot] = (uint32_t)(i + 1); break; }
            }
        }
        free(*htab);
        *htab = nt;
        *hcap = ncap;
    }
    for (size_t j = 0; j < *hcap; j++) {
        const size_t slot = ((size_t)sh + j) % *hcap;
        if ((*htab)[slot] == 0) { (*htab)[slot] = (uint32_t)(n + 1); break; }
    }
    *pn = (uint32_t)(n + 1);
    return (int)n;
#endif /* MICA_IDENT_LINEAR_DEDUP */
}

/* 添加标识符到池（SSO String）。同内容只保留一个（codegen 已去重，此处按内容查重兜底）。
   "System." 前缀压缩为 "@"（运行时规范前缀；@X 唯一 = System.X）——标识符池与 globals
   扁平键统一用 @ 形式，源码编译与 .mlib 反序列化都经此归一。返回池下标；超位宽上限返回 -1。 */
int proto_add_ident(Proto *ch, const char *name, size_t len) {
    char cbuf[512];
    if (len < sizeof(cbuf)) {
        len = str_compress_system(name, len, cbuf);
        name = cbuf;
    }
    return ident_pool_intern(ch, name, len, 0);
}

/* 添加全局标识符到池（SSO String）。与 ident_names 同款 @ 归一 + 去重，但独立配额
   （PROTO_MAX_GLOBAL_IDENTS=65536，LOAD_GLOBAL 的 d 16 位）。返回池下标；超限返回 -1。 */
int proto_add_global_ident(Proto *ch, const char *name, size_t len) {
    char cbuf[512];
    if (len < sizeof(cbuf)) {
        len = str_compress_system(name, len, cbuf);
        name = cbuf;
    }
    return ident_pool_intern(ch, name, len, 1);
}

/* 常量相等（proto_add_const 去重用）：
   - 同 tag 才可能去重：int 0 与 float 0.0 数值相等但类型不同，**不合并**
     （否则 OP_CONST 加载的类型会变，改变运行期语义）。
   - 字符串按内容；其余对象按指针；bool/null 按值。 */
static bool const_eq(Value a, Value b) {
    if (val_is_float(a)) return val_is_float(b) && val_as_float(a) == val_as_float(b);
    if (val_is_float(b)) return false;
    const uint64_t ta = val_tag(a), tb = val_tag(b);
    if (ta != tb) return false;
    switch (ta) {
        case MICA_TNULL: return true;
        case MICA_TFALSE: case MICA_TTRUE: return val_as_bool(a) == val_as_bool(b);
        case MICA_TSTR:
        case MICA_TARRAY:
        case MICA_TMAP:
        case MICA_TINSTANCE:
        case MICA_TCLOSURE:
        case MICA_TOBJ: {
            Obj *oa = val_as_obj(a), *ob = val_as_obj(b);
            if (oa == ob) return true;
            if (oa->type == OBJ_STRING && ob->type == OBJ_STRING)
                return obj_string_eq((const ObjString *)oa, (const ObjString *)ob);
            return false;
        }
        default: return false;
    }
}

int proto_add_const(Proto *ch, Value v) {
    if (ch->n_constants >= PROTO_MAX_CONSTS) return -1; /* 位宽上限 */
    /* 通用池去重：相同 值+类型 复用同一槽（字符串按内容，其余按值/指针）。 */
    for (size_t i = 0; i < ch->n_constants; i++) {
        if (const_eq(ch->constants[i], v)) return (int)i;
    }
    proto_reserve_pool(ch, ch->count, ch->n_constants + 1, ch->n_num_consts, ch->n_str_consts,
                       ch->n_ident_names, ch->n_global_ident_names);
    ch->constants[ch->n_constants++] = v;
    return (int)(ch->n_constants - 1);
}

int proto_add_num_const(Proto *ch, Value v) {
    if (ch->n_num_consts >= PROTO_MAX_CONSTS) return -1; /* 位宽上限 */
    for (size_t i = 0; i < ch->n_num_consts; i++) {
        if (const_eq(ch->num_consts[i], v)) return (int)i;
    }
    proto_reserve_pool(ch, ch->count, ch->n_constants, ch->n_num_consts + 1, ch->n_str_consts,
                       ch->n_ident_names, ch->n_global_ident_names);
    ch->num_consts[ch->n_num_consts++] = v;
    return (int)(ch->n_num_consts - 1);
}

int proto_add_str_const(Proto *ch, Value v) {
    if (ch->n_str_consts >= PROTO_MAX_CONSTS) return -1; /* 位宽上限 */
    for (size_t i = 0; i < ch->n_str_consts; i++) {
        if (const_eq(ch->str_consts[i], v)) return (int)i;
    }
    proto_reserve_pool(ch, ch->count, ch->n_constants, ch->n_num_consts, ch->n_str_consts + 1,
                       ch->n_ident_names, ch->n_global_ident_names);
    ch->str_consts[ch->n_str_consts++] = v;
    return (int)(ch->n_str_consts - 1);
}

void proto_emit(Proto *ch, OpCode op, int a, int b, int c, int line) {
    if (ch->count == ch->cap) {
        size_t ncap = ch->cap ? ch->cap * 2 : 64;
        proto_reserve_pool(ch, ncap, ch->n_constants, ch->n_num_consts, ch->n_str_consts,
                           ch->n_ident_names, ch->n_global_ident_names); /* code 区并入 pool */
    }
    ch->code[ch->count] = instr_encode((int)op, a, b, c);
    ch->lines[ch->count] = line;
    ch->count++;
}

size_t proto_code_len(const Proto *ch) {
    return ch->count;
}

size_t proto_const_count(const Proto *ch) {
    return ch->n_constants;
}

void proto_patch_jump(Proto *ch, size_t instr_idx, int offset) {
    assert(instr_idx < ch->count);
    Instr *in = &ch->code[instr_idx];
    /* 保留 op 与 a（条件寄存器），把 16 位 sBx 写进 d 字段 */
    *in = instr_sbx_set(*in, offset);
}

int proto_line(const Proto *ch, size_t instr_idx) {
    assert(instr_idx < ch->count);
    return ch->lines[instr_idx];
}

/* ---------- opcode 元数据表（阶段 A：统一 name + A/B/C 字段语义） ----------
   新增指令三件事：① proto.h 枚举末尾；② 本表补一行；③ regalloc/vm/disasm 加 case。
   name 供 opcode_name/disasm；字段语义供校验与调试（regalloc 复杂逻辑仍专用）。 */
const InstrDesc k_instr_desc[OP_COUNT] = {
    [OP_CONST] = {"CONST", IARG_REG, IARG_CONST, IARG_UNUSED},
    [OP_KNUM] = {"KNUM", IARG_REG, IARG_CONST, IARG_UNUSED},
    [OP_KSTR] = {"KSTR", IARG_REG, IARG_CONST, IARG_UNUSED},
    [OP_NULL] = {"NULL", IARG_REG, IARG_UNUSED, IARG_UNUSED},
    [OP_TRUE] = {"TRUE", IARG_REG, IARG_UNUSED, IARG_UNUSED},
    [OP_FALSE] = {"FALSE", IARG_REG, IARG_UNUSED, IARG_UNUSED},
    [OP_MOV] = {"MOV", IARG_REG, IARG_REG, IARG_UNUSED},
    [OP_MOVI] = {"MOVI", IARG_REG, IARG_SBX, IARG_UNUSED},
    [OP_LOAD_GLOBAL] = {"LOAD_GLOBAL", IARG_REG, IARG_IDENT, IARG_UNUSED},
    [OP_STORE_GLOBAL] = {"STORE_GLOBAL", IARG_REG, IARG_IDENT, IARG_UNUSED},
    [OP_GET_MEMBER] = {"GET_MEMBER", IARG_REG, IARG_IDENT, IARG_REG},
    [OP_SET_MEMBER] = {"SET_MEMBER", IARG_REG, IARG_IDENT, IARG_REG},
    [OP_INDEX] = {"INDEX", IARG_REG, IARG_REG, IARG_REG},
    [OP_INDEX_SET] = {"INDEX_SET", IARG_REG, IARG_REG, IARG_REG},
    [OP_NEG] = {"NEG", IARG_REG, IARG_REG, IARG_UNUSED},
    [OP_NOT] = {"NOT", IARG_REG, IARG_REG, IARG_UNUSED},
    [OP_ADD] = {"ADD", IARG_REG, IARG_REG, IARG_REG},
    [OP_SUB] = {"SUB", IARG_REG, IARG_REG, IARG_REG},
    [OP_MUL] = {"MUL", IARG_REG, IARG_REG, IARG_REG},
    [OP_DIV] = {"DIV", IARG_REG, IARG_REG, IARG_REG},
    [OP_IDIV] = {"IDIV", IARG_REG, IARG_REG, IARG_REG},
    [OP_MOD] = {"MOD", IARG_REG, IARG_REG, IARG_REG},
    [OP_POW] = {"POW", IARG_REG, IARG_REG, IARG_REG},
    [OP_EQ] = {"EQ", IARG_REG, IARG_REG, IARG_REG},
    [OP_NE] = {"NE", IARG_REG, IARG_REG, IARG_REG},
    [OP_LT] = {"LT", IARG_REG, IARG_REG, IARG_REG},
    [OP_LE] = {"LE", IARG_REG, IARG_REG, IARG_REG},
    [OP_ISEQ] = {"ISEQ", IARG_REG, IARG_REG, IARG_UNUSED},
    [OP_ISNE] = {"ISNE", IARG_REG, IARG_REG, IARG_UNUSED},
    [OP_ISLT] = {"ISLT", IARG_REG, IARG_REG, IARG_UNUSED},
    [OP_ISLE] = {"ISLE", IARG_REG, IARG_REG, IARG_UNUSED},
    [OP_ISLT_LEN] = {"ISLT_LEN", IARG_REG, IARG_REG, IARG_UNUSED},
    [OP_JMP] = {"JMP", IARG_UNUSED, IARG_SBX, IARG_UNUSED},
    [OP_JMP_IF_FALSE] = {"JMP_IF_FALSE", IARG_REG, IARG_SBX, IARG_UNUSED},
    [OP_JMP_IF_TRUE] = {"JMP_IF_TRUE", IARG_REG, IARG_SBX, IARG_UNUSED},
    [OP_ARRAY] = {"ARRAY", IARG_REG, IARG_IMM, IARG_REG},
    [OP_MAP] = {"MAP", IARG_REG, IARG_IMM, IARG_REG},
    [OP_ERROR] = {"ERROR", IARG_CONST, IARG_UNUSED, IARG_UNUSED},
    [OP_GET_CAPVAL] = {"GET_CAPVAL", IARG_REG, IARG_IMM, IARG_UNUSED},
    [OP_SET_CAPVAL] = {"SET_CAPVAL", IARG_REG, IARG_IMM, IARG_UNUSED},
    [OP_CLOSURE] = {"CLOSURE", IARG_REG, IARG_CONST, IARG_IMM},
    [OP_VARARGS] = {"VARARGS", IARG_REG, IARG_UNUSED, IARG_UNUSED},
    [OP_CALL] = {"CALL", IARG_REG, IARG_IMM, IARG_RESULT},
    [OP_FASTCALL] = {"FASTCALL", IARG_REG, IARG_IMM, IARG_RESULT},
    [OP_RETURN] = {"RETURN", IARG_REG, IARG_UNUSED, IARG_RESULT},
    [OP_NEW_ERROR] = {"NEW_ERROR", IARG_REG, IARG_REG, IARG_UNUSED},
    [OP_NEW_ERROR_NAMED] = {"NEW_ERROR_NAMED", IARG_REG, IARG_IMM, IARG_REG},
    [OP_ERROR_INFO] = {"ERROR_INFO", IARG_CONST, IARG_CONST, IARG_UNUSED},
    [OP_IS_ERROR] = {"IS_ERROR", IARG_REG, IARG_REG, IARG_UNUSED},
    [OP_THROW] = {"THROW", IARG_REG, IARG_UNUSED, IARG_UNUSED},
    [OP_MATCH_ERROR] = {"MATCH_ERROR", IARG_REG, IARG_REG, IARG_UNUSED},
    [OP_GET_ERROR_PAYLOAD] = {"GET_ERROR_PAYLOAD", IARG_REG, IARG_REG, IARG_IMM},
    [OP_GET_METHOD] = {"GET_METHOD", IARG_REG, IARG_IDENT, IARG_REG},
    [OP_YIELD] = {"YIELD", IARG_REG, IARG_REG, IARG_UNUSED},
    [OP_DEFER_MARK] = {"DEFER_MARK", IARG_UNUSED, IARG_UNUSED, IARG_UNUSED},
    [OP_DEFER_PUSH] = {"DEFER_PUSH", IARG_REG, IARG_UNUSED, IARG_UNUSED},
    [OP_DEFER_RUN] = {"DEFER_RUN", IARG_UNUSED, IARG_UNUSED, IARG_UNUSED},
    [OP_NEW_ENUM_NAMED] = {"NEW_ENUM_NAMED", IARG_REG, IARG_IMM, IARG_REG},
    [OP_ENUM_INFO] = {"ENUM_INFO", IARG_CONST, IARG_CONST, IARG_UNUSED},
    [OP_GET_ENUM_PAYLOAD] = {"GET_ENUM_PAYLOAD", IARG_REG, IARG_REG, IARG_IMM},
    [OP_MATCH_ENUM] = {"MATCH_ENUM", IARG_REG, IARG_REG, IARG_CONST},
    [OP_MATCH_LITERAL] = {"MATCH_LITERAL", IARG_REG, IARG_REG, IARG_CONST},
    [OP_IS_TYPE] = {"IS_TYPE", IARG_REG, IARG_REG, IARG_CONST},
    [OP_CONCAT] = {"CONCAT", IARG_REG, IARG_REG, IARG_IMM},
    [OP_LENGTH] = {"LENGTH", IARG_REG, IARG_REG, IARG_UNUSED},
    [OP_ADDVN] = {"ADDVN", IARG_REG, IARG_REG, IARG_CONST},
    [OP_SUBVN] = {"SUBVN", IARG_REG, IARG_REG, IARG_CONST},
    [OP_MULVN] = {"MULVN", IARG_REG, IARG_REG, IARG_CONST},
    [OP_DIVVN] = {"DIVVN", IARG_REG, IARG_REG, IARG_CONST},
    [OP_IDIVVN] = {"IDIVVN", IARG_REG, IARG_REG, IARG_CONST},
    [OP_MODVN] = {"MODVN", IARG_REG, IARG_REG, IARG_CONST},
    [OP_ADDNV] = {"ADDNV", IARG_REG, IARG_CONST, IARG_REG},
    [OP_MULNV] = {"MULNV", IARG_REG, IARG_CONST, IARG_REG},
    [OP_INC] = {"INC", IARG_REG, IARG_UNUSED, IARG_UNUSED},
    [OP_DEC] = {"DEC", IARG_REG, IARG_UNUSED, IARG_UNUSED},
    [OP_RET0] = {"RET0", IARG_UNUSED, IARG_UNUSED, IARG_UNUSED},
    [OP_RET1] = {"RET1", IARG_REG, IARG_UNUSED, IARG_UNUSED},
    [OP_FORI] = {"FORI", IARG_REG, IARG_SBX, IARG_UNUSED},
    [OP_FORL] = {"FORL", IARG_REG, IARG_SBX, IARG_UNUSED},
    [OP_EXIT] = {"EXIT", IARG_UNUSED, IARG_UNUSED, IARG_UNUSED},
    [OP_TAILCALL] = {"TAILCALL", IARG_REG, IARG_IMM, IARG_RESULT},
    [OP_GLOAD_INDEX] = {"GLOAD_INDEX", IARG_REG, IARG_IDENT, IARG_REG},
    [OP_ISEQN] = {"ISEQN", IARG_REG, IARG_CONST, IARG_UNUSED},
    [OP_ISNEN] = {"ISNEN", IARG_REG, IARG_CONST, IARG_UNUSED},
    [OP_ISEQS] = {"ISEQS", IARG_REG, IARG_CONST, IARG_UNUSED},
    [OP_ISNES] = {"ISNES", IARG_REG, IARG_CONST, IARG_UNUSED},
    [OP_GETI] = {"GETI", IARG_REG, IARG_REG, IARG_CONST},
    [OP_SETI] = {"SETI", IARG_REG, IARG_REG, IARG_CONST},
    [OP_ADDI] = {"ADDI", IARG_REG, IARG_REG, IARG_IMM},
    [OP_SUBI] = {"SUBI", IARG_REG, IARG_REG, IARG_IMM},
};

const char *opcode_name(OpCode op) {
    if (op >= OP_COUNT) return "<?>";
    return k_instr_desc[op].name;
}

void proto_disassemble(const Proto *ch, const char *name) {
    printf("== %s ==\n", name);
    for (size_t i = 0; i < ch->count; i++) {
        const Instr in = ch->code[i];
        const OpCode op = instr_op(in);
        const int a = IS_AD_OP(op) ? instr_a_ad(in) : instr_a(in);
        const int b = instr_b(in), c = instr_c(in);
        printf("%4zu  %-4d  %-13s", i, ch->lines[i], opcode_name(op));
        switch (op) {
            case OP_JMP:
                printf("  %+d", instr_sbx(in));
                break;
            case OP_JMP_IF_FALSE:
            case OP_JMP_IF_TRUE:
                printf("  a=%d sBx=%+d", a, instr_sbx(in));
                break;
            case OP_CONST:
                printf("  a=%d C[%d]=", a, instr_bx(in));
                value_print(stdout, ch->constants[instr_bx(in)]);
                break;
            case OP_KNUM:
                printf("  a=%d N[%d]=", a, instr_bx(in));
                value_print(stdout, ch->num_consts[instr_bx(in)]);
                break;
            case OP_KSTR:
                printf("  a=%d S[%d]=", a, instr_bx(in));
                value_print(stdout, ch->str_consts[instr_bx(in)]);
                break;
            case OP_LOAD_GLOBAL:
            case OP_STORE_GLOBAL:
                printf("  a=%d d=%d GID[%d]=%.*s", a, instr_bx(in), instr_bx(in),
                       (int)str_len(&ch->global_ident_names[instr_bx(in)]),
                       str_data(&ch->global_ident_names[instr_bx(in)]));
                break;
            case OP_GET_MEMBER:
            case OP_SET_MEMBER:
                printf("  a=%d c=%d ID[%d]=%.*s", a, c, b, (int)str_len(&ch->ident_names[b]),
                       str_data(&ch->ident_names[b]));
                break;
            case OP_GET_METHOD:
                printf("  a=%d c=%d ID[%d]=%.*s", a, c, b, (int)str_len(&ch->ident_names[b]),
                       str_data(&ch->ident_names[b]));
                break;
            case OP_ARRAY:
            case OP_MAP:
                printf("  a=%d b=%d(个数) c=%d(首寄存器)", a, b, c);
                break;
            case OP_CONCAT:
                printf("  a=%d c=%d(首寄存器) b=%d(段数=%d)", a, b, c, c + 1);
                break;
            case OP_LENGTH:
                printf("  a=%d b=%d", a, b);
                break;
            case OP_EQ: case OP_NE: case OP_LT: case OP_LE: /* 比较族 ABC */
                printf("  a=%d b=%d c=%d", a, b, c);
                break;
            case OP_ISEQ: case OP_ISNE: case OP_ISLT: case OP_ISLE:
            case OP_ISLT_LEN: /* 比较族 ABC */
                printf("  a=%d b=%d (真则跳过下一条)", a, b);
                break;
            case OP_ISEQN: case OP_ISNEN: /* 数值常量比较（AD，num 池） */
                printf("  a=%d N[%d]=", a, instr_bx(in));
                if (instr_bx(in) < (int)ch->n_num_consts)
                    value_print(stdout, ch->num_consts[instr_bx(in)]);
                else fputs("?", stdout);
                printf(" (真则跳过下一条)");
                break;
            case OP_ISEQS: case OP_ISNES: /* 字符串常量比较（AD，str 池） */
                printf("  a=%d S[%d]=", a, instr_bx(in));
                if (instr_bx(in) < (int)ch->n_str_consts)
                    value_print(stdout, ch->str_consts[instr_bx(in)]);
                else fputs("?", stdout);
                printf(" (真则跳过下一条)");
                break;
            case OP_GETI:
                printf("  a=%d b=%d N[%d]=%.0f", a, b, c, c < (int)ch->n_num_consts ? val_as_float(ch->num_consts[c]) : 0.0);
                break;
            case OP_SETI:
                printf("  a=%d b=%d N[%d]=%.0f", a, b, c, c < (int)ch->n_num_consts ? val_as_float(ch->num_consts[c]) : 0.0);
                break;
            case OP_ADDI:
                printf("  a=%d b=%d imm=%d", a, b, (int8_t)c);
                break;
            case OP_SUBI:
                printf("  a=%d b=%d imm=%d", a, b, (int8_t)c);
                break;
            case OP_CLOSURE:
                printf("  a=%d b=%d c=%d(capvals)", a, b, c);
                /* 捕获描述在 proto->capvals：L=外层局部槽, C=外层闭包 capval 下标 */
                for (size_t u = 0; u < ch->n_capvals && u < (size_t)c; u++) {
                    printf(" %s%d", ch->capvals[u].is_local ? "L" : "C", ch->capvals[u].index);
                }
                break;
            case OP_ERROR:
                printf("  a=%d C[%d]=", a, a);
                value_print(stdout, ch->constants[a]);
                break;
            case OP_ERROR_INFO:
                printf("  a=C[%d]=", a);
                if (a < (int)ch->n_constants) value_print(stdout, ch->constants[a]);
                else fputs("null", stdout);
                printf(" b=C[%d]=", b);
                if (b < (int)ch->n_constants) value_print(stdout, ch->constants[b]);
                else fputs("null", stdout);
                break;
            case OP_ENUM_INFO:
                printf("  a=C[%d]=", a);
                if (a < (int)ch->n_constants) value_print(stdout, ch->constants[a]);
                else fputs("null", stdout);
                printf(" b=C[%d]=", b);
                if (b < (int)ch->n_constants) value_print(stdout, ch->constants[b]);
                else fputs("null", stdout);
                break;
            case OP_MATCH_LITERAL:
            case OP_IS_TYPE:
                printf("  a=%d b=%d C[%d]=", a, b, c);
                if (c < (int)ch->n_constants) value_print(stdout, ch->constants[c]);
                else fputs("null", stdout);
                break;
            case OP_MOVI:
                printf("  a=%d sBx=%d", a, instr_sbx(in));
                break;
            case OP_ADDVN: case OP_SUBVN: case OP_MULVN: case OP_DIVVN: case OP_IDIVVN: case OP_MODVN: {
                printf("  a=%d b=%d N[%d]=", a, b, c); /* ABC：b=源寄存器 8 位、c=数值常量下标 8 位 */
                if (c < (int)ch->n_num_consts) value_print(stdout, ch->num_consts[c]);
                else fputs("null", stdout);
                break;
            }
            case OP_ADDNV: case OP_MULNV:
                printf("  a=%d N[%d]=", a, b);
                if (b < (int)ch->n_num_consts) value_print(stdout, ch->num_consts[b]);
                else fputs("null", stdout);
                printf(" c=%d", c);
                break;
            case OP_INC: case OP_DEC:
                printf("  a=%d", a);
                break;
            case OP_RET0:
                fputs("  返回0个值", stdout);
                break;
            case OP_RET1:
                printf("  返回 R%d", a);
                break;
            case OP_FORI:
            case OP_FORL:
                printf("  a=%d(控制组) sBx=%+d", a, instr_sbx(in));
                break;
            case OP_EXIT:
                printf("  退出 VM");
                break;
            default:
                printf("  a=%d b=%d c=%d", a, b, c);
                break;
        }
        putchar('\n');
    }
    if (ch->n_ident_names > 0) {
        printf("  标识符池 (%zu):\n", ch->n_ident_names);
        for (size_t i = 0; i < ch->n_ident_names; i++) {
            printf("    ID[%3zu] = %.*s\n", i, (int)str_len(&ch->ident_names[i]),
                   str_data(&ch->ident_names[i]));
        }
    }
    if (ch->n_global_ident_names > 0) {
        printf("  全局标识符池 (%zu):\n", ch->n_global_ident_names);
        for (size_t i = 0; i < ch->n_global_ident_names; i++) {
            printf("    GID[%3zu] = %.*s\n", i, (int)str_len(&ch->global_ident_names[i]),
                   str_data(&ch->global_ident_names[i]));
        }
    }
}

bool proto_validate(const Proto *ch) {
    /* 寄存器窗口上限：VM 帧只分配 max_regs 个槽，任何下标 >= max_regs 的寄存器访问都会
       越过本帧读到相邻帧/未分配内存。**.mlib 反序列化后必须过此校验**（畸形字节码
       过去完全不被检查，越界读写是内存安全事件而非错误提示）。
       注：max_regs 由 regalloc 后处理**压缩**，可能小于代码里的最大下标，因此这里
       只在"压缩后的窗口确实容不下某条指令的操作数"时报错——合法字节码不会触发。
       排查：设 MICA_VALIDATE_DEBUG=1 会在首次失败时报出指令下标与原因。 */
    const bool dbg = getenv("MICA_VALIDATE_DEBUG") != NULL;
#define VFAIL(idx, ...)                                                       \
    do {                                                                      \
        if (dbg) {                                                            \
            fprintf(stderr, "proto_validate FAIL at i=%zu: ", (size_t)(idx)); \
            fprintf(stderr, __VA_ARGS__);                                     \
            fprintf(stderr, "\n");                                            \
        }                                                                     \
        return false;                                                         \
    } while (0)
    const int nr = (int)ch->max_regs;
    if (ch->max_regs > PROTO_MAX_REGS)
        VFAIL(0, "max_regs=%d > %d", (int)ch->max_regs, PROTO_MAX_REGS);
    /* nr==0：没有任何寄存器窗口（如手工构造的只有 OP_RET0 的 proto）。此时不做寄存器
       校验——否则单元测试里那些"零寄存器 + 常量/跳转检查"的构造会被误杀。真实函数
       的 max_regs ≥ arity+1。 */
#define REG_OK(r)                                                                       \
    ((nr == 0 || ((r) >= 0 && (r) < nr))                                                \
         ? 1                                                                            \
         : (dbg ? (fprintf(stderr, "REG_FAIL i=%zu op=%d r=%d nr=%d a=%d b=%d c=%d\n",  \
                           (size_t)i, (int)op, (int)(r), nr, a, b, c),                  \
                  0)                                                                   \
               : 0))
    for (size_t i = 0; i < ch->count; i++) {
        const Instr in = ch->code[i];
        const OpCode op = instr_op(in);
        const int a = IS_AD_OP(op) ? instr_a_ad(in) : instr_a(in);
        const int b = instr_b(in), c = instr_c(in);
        if (op >= OP_COUNT) return false;
        /* 逐指令的寄存器/结果模式校验。
           operand 语义取自 opcodes.h 的注释与 k_instr_desc；凡本表未列出的 op 不做
           寄存器检查（保守：宁放过可疑，不误杀合法字节码）。 */
        switch (op) {
            /* --- 单目标写 + 单/双寄存器读 --- */
            case OP_MOV:
                if (!REG_OK(a) || !REG_OK(b)) return false;
                break;
            case OP_INDEX:
                if (!REG_OK(a) || !REG_OK(b) || !REG_OK(c)) return false;
                break;
            case OP_INDEX_SET:
                if (!REG_OK(a) || !REG_OK(b) || !REG_OK(c)) return false;
                break;
            case OP_GLOAD_INDEX:
                if (!REG_OK(a) || !REG_OK(c)) return false;
                break;
            case OP_ADDI: case OP_SUBI:
                if (!REG_OK(a) || !REG_OK(b)) return false;
                break;
            case OP_GETI:
                if (!REG_OK(a) || !REG_OK(b)) return false;
                break;
            case OP_SETI:
                if (!REG_OK(a) || !REG_OK(b)) return false;
                break;
            case OP_CONST: case OP_KNUM: case OP_KSTR: case OP_MOVI:
            case OP_NULL: case OP_TRUE: case OP_FALSE:
                if (!REG_OK(a)) return false;
                break;
            /* --- 算术 / 比较（ABC：a 目标，b/c 源） --- */
            case OP_ADD: case OP_SUB: case OP_MUL: case OP_DIV: case OP_IDIV: case OP_MOD:
            case OP_POW:
                if (!REG_OK(a) || !REG_OK(b) || !REG_OK(c)) return false;
                break;
            case OP_NEG: case OP_NOT:
                if (!REG_OK(a) || !REG_OK(b)) return false;
                break;
            case OP_ADDVN: case OP_SUBVN: case OP_MULVN: case OP_DIVVN: case OP_IDIVVN:
            case OP_MODVN:
            case OP_ADDNV: case OP_MULNV:
                if (!REG_OK(a) || !REG_OK(b)) return false;
                break;
            case OP_INC: case OP_DEC:
                if (!REG_OK(a)) return false;
                break;
            case OP_EQ: case OP_NE: case OP_LT: case OP_LE:
            case OP_ISLT: case OP_ISLE: case OP_ISEQ: case OP_ISNE:
                if (!REG_OK(a) || !REG_OK(b) || !REG_OK(c)) return false;
                break;
            case OP_ISLT_LEN:
                if (!REG_OK(a) || !REG_OK(b)) return false;
                break;
            case OP_ISEQN: case OP_ISNEN: case OP_ISEQS: case OP_ISNES:
                if (!REG_OK(a)) return false;
                break;
            /* --- 成员/长度 --- */
            case OP_GET_MEMBER: case OP_SET_MEMBER:
                if (!REG_OK(a) || !REG_OK(c)) return false;
                break;
            case OP_GET_METHOD:
                /* 方法调用预备：a=目标、c=receiver */
                if (!REG_OK(a) || !REG_OK(c)) return false;
                break;
            case OP_LENGTH:
                if (!REG_OK(a) || !REG_OK(b)) return false;
                break;
            /* --- 控制流（a 为条件寄存器；AD 下用 instr_a_ad） --- */
            case OP_JMP_IF_FALSE: case OP_JMP_IF_TRUE:
                if (!REG_OK(a)) return false;
                break;
            case OP_FORI: case OP_FORL:
                /* 数值 for 用连续 4 槽 [a, a+3] */
                if (!REG_OK(a) || !REG_OK(a + 3)) return false;
                break;
            /* --- 调用/返回：a 起的结果窗口必须落在帧内；结果模式受 c 字段约束 --- */
            case OP_CALL: case OP_FASTCALL: {
                const int mode = c & RESULT_MASK;
                const int nres = (mode == RESULT_VARARGS) ? 1 : mode;
                if (!REG_OK(a) || !REG_OK(b)) return false;
                /* 实参窗口：slot0=函数、slot1=self、实参 2..1+argc → 最高寄存器 a+b。
                   结果窗口：a .. a+nres-1。 */
                if (!REG_OK(a + b)) return false;
                if (nres > 0 && !REG_OK(a + nres - 1)) return false;
                if (c != (c & (RESULT_MASK | RESULT_PROTECT))) return false; /* 未定义高位 */
                break;
            }
            case OP_TAILCALL:
                if (!REG_OK(a) || !REG_OK(b)) return false;
                if (!REG_OK(a + b)) return false;
                break;
            case OP_RET0:
                break;
            case OP_RET1:
                if (!REG_OK(a)) return false;
                break;
            case OP_RETURN: {
                /* c=0（零结果）时 a 未用；c=127(VARARGS) 时结果区长度运行期决定，
                   只校验起始槽。 */
                const int actual = (c == RESULT_VARARGS) ? 1 : (int)c;
                if (actual > 0 && !REG_OK(a)) return false;
                if (actual > 1 && !REG_OK(a + actual - 1)) return false;
                break;
            }
            /* --- 闭包/捕获/varargs --- */
            case OP_CLOSURE:
                if (!REG_OK(a)) return false;
                break;
            case OP_GET_CAPVAL:
                if (!REG_OK(a) || b >= 256) return false;
                break;
            case OP_SET_CAPVAL:
                if (!REG_OK(a) || b >= 256) return false;
                break;
            case OP_VARARGS:
                /* c = 需要的值个数；0 表示不带值的形式（手工字节码/边界用例允许）。 */
                if (!REG_OK(a)) return false;
                if (c > 0 && !REG_OK(a + c - 1)) return false;
                break;
            /* --- 字符串/构造（连续区间） --- */
            case OP_CONCAT:
                if (!REG_OK(a) || !REG_OK(b)) return false;
                if (!REG_OK(b + c)) return false;               /* c = 段数-1 */
                break;
            case OP_ARRAY:
                /* 元素连续区：c .. c+b-1（b = 元素个数，vm_cold_array） */
                if (!REG_OK(a) || !REG_OK(c)) return false;
                if (b > 0 && !REG_OK(c + b - 1)) return false;
                break;
            case OP_MAP:
                /* 键值对连续区：c .. c+2b-1（b = 键值对个数，vm_cold_map） */
                if (!REG_OK(a) || !REG_OK(c)) return false;
                if (b > 0 && !REG_OK(c + 2 * b - 1)) return false;
                break;
            /* --- 错误/枚举（常量池下标已在下方校验；这里只管寄存器） --- */
            case OP_ERROR: case OP_NEW_ERROR:
                if (!REG_OK(a) || !REG_OK(b)) return false;
                break;
            case OP_NEW_ERROR_NAMED: case OP_MATCH_ERROR:
                if (!REG_OK(a) || !REG_OK(b)) return false;
                break;
            case OP_GET_ERROR_PAYLOAD: case OP_NEW_ENUM_NAMED: case OP_MATCH_ENUM:
            case OP_MATCH_LITERAL: case OP_IS_TYPE:
                if (!REG_OK(a) || !REG_OK(b) || !REG_OK(c)) return false;
                break;
            case OP_ERROR_INFO: case OP_ENUM_INFO:
                break; /* 描述符字：寄存器语义由消费它的指令覆盖 */
            case OP_IS_ERROR:
                if (!REG_OK(a) || !REG_OK(b)) return false;
                break;
            case OP_THROW:
                if (!REG_OK(a)) return false;
                break;
            /* --- 协程/defer --- */
            case OP_YIELD:
                if (!REG_OK(b)) return false;
                break;
            case OP_DEFER_MARK: case OP_DEFER_RUN:
                break;
            case OP_DEFER_PUSH:
                if (!REG_OK(a)) return false;
                break;
            case OP_EXIT:
                break;
            case OP_JMP:
                break;
            default:
                break;
        }
        switch (op) {
            case OP_CONST:
                if (instr_bx(in) >= (int)ch->n_constants) return false;
                break;
            case OP_KNUM:
                if (instr_bx(in) >= (int)ch->n_num_consts) return false;
                break;
            case OP_KSTR:
                if (instr_bx(in) >= (int)ch->n_str_consts) return false;
                break;
            case OP_LOAD_GLOBAL:
            case OP_STORE_GLOBAL:
                if (instr_bx(in) >= (int)ch->n_global_ident_names) return false; /* AD：d=全局标识符池下标 */
                break;
            case OP_GET_MEMBER:
            case OP_SET_MEMBER:
            case OP_GET_METHOD:
                if (b >= (int)ch->n_ident_names) return false; /* ABC：b=成员标识符池下标 8 位 */
                break;
            case OP_GLOAD_INDEX:
                if (b >= (int)ch->n_global_ident_names) return false; /* ABC：b=全局标识符池下标 */
                break;
            case OP_ISEQN: case OP_ISNEN:
                if (instr_bx(in) >= (int)ch->n_num_consts) return false; /* AD：d=数值常量下标 */
                break;
            case OP_ISEQS: case OP_ISNES:
                if (instr_bx(in) >= (int)ch->n_str_consts) return false; /* AD：d=字符串常量下标 */
                break;
            case OP_GETI: case OP_SETI:
                if (c >= (int)ch->n_num_consts) return false; /* c=数值常量下标 */
                break;
            case OP_ADDVN: case OP_SUBVN: case OP_MULVN: case OP_DIVVN: case OP_IDIVVN: case OP_MODVN:
                if (c >= (int)ch->n_num_consts) return false; /* c=数值常量下标 */
                break;
            case OP_ADDNV: case OP_MULNV:
                if (b >= (int)ch->n_num_consts) return false; /* b=数值常量下标 */
                break;
            case OP_ERROR:
                if (a >= (int)ch->n_constants) return false;
                break;
            case OP_ERROR_INFO:
                if ((a > 0 && a >= (int)ch->n_constants) ||
                    (b > 0 && b >= (int)ch->n_constants)) return false;
                break;
            case OP_ENUM_INFO:
                if ((a > 0 && a >= (int)ch->n_constants) ||
                    (b > 0 && b >= (int)ch->n_constants)) return false;
                break;
            case OP_MATCH_LITERAL:
            case OP_IS_TYPE:
                if (c >= (int)ch->n_constants) return false;
                break;
            case OP_CLOSURE:
                /* c=创建的 lambda/协程的捕获数（子 proto->n_capvals），与 ch->n_capvals
                   （当前函数捕获的外层变量数）无关，不能比较。子 proto 的捕获描述由
                   OP_CLOSURE 运行期从常量池读取，此处只保证 b 是合法常量即可。 */
                if (b >= (int)ch->n_constants) return false;
                break;
            case OP_JMP:
            case OP_JMP_IF_FALSE:
            case OP_JMP_IF_TRUE:
            case OP_FORI:
            case OP_FORL: {
                const long target = (long)i + 1 + instr_sbx(in);
                if (target < 0 || (size_t)target >= ch->count) return false;
                break;
            }
            default:
                break;
        }
    }
#undef REG_OK
#undef RVAL_OK
    return true;
}
