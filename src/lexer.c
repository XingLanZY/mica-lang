#include "lexer.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct Lexer {
    const char *src;
    size_t len;
    size_t pos;
    int line;
    int col;
    bool newline_pending;
    int nl_line;
    int nl_col;
    const char *nl_start;
    bool errored;
    LexError err;
};

/* ---------- 字符分类 ---------- */

static bool is_digit(char c) { return c >= '0' && c <= '9'; }

static bool is_ident_start(char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
}

static bool is_ident_part(char c) { return is_ident_start(c) || is_digit(c); }

/* ---------- 错误 ---------- */

static void lex_error_at(Lexer *lx, int line, int col, int span, const char *msg) {
    if (lx->err.has_error) return;
    lx->err.has_error = true;
    lx->err.line = line;
    lx->err.col = col;
    lx->err.span = span > 0 ? span : 1;
    snprintf(lx->err.message, sizeof(lx->err.message), "%s", msg);
    lx->errored = true;
}

/* ---------- 位置推进 ---------- */

/* 前进一个字节并同步行列；\r、\n、\r\n 都算一次换行。 */
static void lex_advance_byte(Lexer *lx) {
    char c = lx->src[lx->pos];
    if (c == '\n') {
        lx->line++;
        lx->col = 1;
        lx->pos++;
    } else if (c == '\r') {
        lx->line++;
        lx->col = 1;
        lx->pos++;
        if (lx->pos < lx->len && lx->src[lx->pos] == '\n') lx->pos++;
    } else {
        lx->col++;
        lx->pos++;
    }
}

/* 记录首个待发换行（连续换行折叠为一个 NEWLINE token）。 */
static void lex_mark_newline(Lexer *lx) {
    if (!lx->newline_pending) {
        lx->newline_pending = true;
        lx->nl_line = lx->line;
        lx->nl_col = lx->col;
        lx->nl_start = lx->src + lx->pos;
    }
}

/* ---------- 空白与注释 ---------- */

static void lex_skip_block_comment(Lexer *lx) {
    int start_line = lx->line, start_col = lx->col;
    lx->pos += 2;
    lx->col += 2;
    while (lx->pos < lx->len) {
        char c = lx->src[lx->pos];
        if (c == '*' && lx->pos + 1 < lx->len && lx->src[lx->pos + 1] == '/') {
            lx->pos += 2;
            lx->col += 2;
            return;
        }
        if (c == '\n' || c == '\r') {
            lex_mark_newline(lx);
            lex_advance_byte(lx);
            continue;
        }
        lx->pos++;
        lx->col++;
    }
    lex_error_at(lx, start_line, start_col, 2, "unterminated block comment");
}

/* 跳过空白、注释；期间遇到的换行记入 newline_pending。 */
static void lex_skip_trivia(Lexer *lx) {
    for (;;) {
        if (lx->pos >= lx->len) return;
        char c = lx->src[lx->pos];
        if (c == ' ' || c == '\t') {
            lx->pos++;
            lx->col++;
            continue;
        }
        if (c == '\n' || c == '\r') {
            lex_mark_newline(lx);
            lex_advance_byte(lx);
            continue;
        }
        if (c == '#') {
            while (lx->pos < lx->len && lx->src[lx->pos] != '\n' && lx->src[lx->pos] != '\r') {
                lx->pos++;
                lx->col++;
            }
            continue;
        }
        if (c == '/' && lx->pos + 1 < lx->len && lx->src[lx->pos + 1] == '*') {
            lex_skip_block_comment(lx);
            if (lx->errored) return;
            continue;
        }
        return;
    }
}

/* ---------- 数字 ---------- */

static bool digit_for_base(char c, int base) {
    switch (base) {
        case 16:
            return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
        case 2:
            return c == '0' || c == '1';
        case 8:
            return c >= '0' && c <= '7';
        default:
            return c >= '0' && c <= '9';
    }
}

/* 扫描一串指定位数的数字，允许用单引号分隔（仅限数字之间，如 1'000'000）。 */
static bool scan_digits(Lexer *lx, size_t *pp, int base) {
    bool any = false;
    while (*pp < lx->len) {
        char c = lx->src[*pp];
        if (c == '\'') {
            if (!any) return false;
            if (*pp + 1 >= lx->len) return false;
            if (!digit_for_base(lx->src[*pp + 1], base)) return false;
            (*pp)++;
            continue;
        }
        if (digit_for_base(c, base)) {
            any = true;
            (*pp)++;
            continue;
        }
        break;
    }
    return any;
}

static bool lex_number(Lexer *lx, Token *tok) {
    size_t p = lx->pos;
    int base = 10;

    if (lx->src[p] == '0' && p + 1 < lx->len) {
        char nx = lx->src[p + 1];
        if (nx == 'x' || nx == 'X') {
            base = 16;
            p += 2;
        } else if (nx == 'b' || nx == 'B') {
            base = 2;
            p += 2;
        } else if (nx == 'o' || nx == 'O') {
            base = 8;
            p += 2;
        }
    }

    if (base == 10) {
        if (!scan_digits(lx, &p, 10)) goto invalid;

        /* 小数部分：'.' 后必须紧跟数字，且不能是 '..'（区间运算符）。 */
        if (p < lx->len && lx->src[p] == '.' &&
            !(p + 1 < lx->len && lx->src[p + 1] == '.')) {
            size_t save = p;
            p++;
            if (!scan_digits(lx, &p, 10)) {
                p = save; /* 形如 "1." 退回，'.' 交由运算符处理 */
            }
        }

        /* 指数部分：e/E [+-] 数字。 */
        if (p < lx->len && (lx->src[p] == 'e' || lx->src[p] == 'E')) {
            size_t q = p + 1;
            if (q < lx->len && (lx->src[q] == '+' || lx->src[q] == '-')) q++;
            if (q < lx->len && is_digit(lx->src[q])) {
                p++;
                if (p < lx->len && (lx->src[p] == '+' || lx->src[p] == '-')) p++;
                scan_digits(lx, &p, 10);
            }
        }
    } else {
        if (!scan_digits(lx, &p, base)) goto invalid;
    }

    /* 尾部合法性：禁止 "0b102"、"123abc"、"1.2.3" 这类非法拼接。 */
    if (p < lx->len) {
        char c = lx->src[p];
        if (is_digit(c)) goto invalid;
        if (is_ident_start(c)) goto invalid;
        if (c == '.' && p + 1 < lx->len && is_digit(lx->src[p + 1])) goto invalid;
    }

    tok->kind = TOK_NUMBER;
    tok->len = p - lx->pos;
    lx->pos = p;
    lx->col += (int)tok->len;
    return true;

invalid:
    lex_error_at(lx, tok->line, tok->col, (int)(p - lx->pos), "invalid number literal");
    return false;
}

/* ---------- 标识符 / 关键字 ---------- */

static void lex_ident(Lexer *lx, Token *tok) {
    size_t p = lx->pos;
    while (p < lx->len && is_ident_part(lx->src[p])) p++;
    size_t n = p - lx->pos;
    TokenKind kw = token_keyword_lookup(lx->src + lx->pos, n);
    tok->kind = (kw == TOK_EOF) ? TOK_IDENT : kw;
    tok->len = n;
    lx->pos = p;
    lx->col += (int)n;

    /* try 紧跟 ? 或 !（无空白）时合并为 try? / try!。 */
    if (kw == TOK_TRY && lx->pos < lx->len) {
        char c = lx->src[lx->pos];
        if (c == '?') {
            tok->kind = TOK_TRY_QUESTION;
            tok->len++;
            lx->pos++;
            lx->col++;
        } else if (c == '!') {
            tok->kind = TOK_TRY_BANG;
            tok->len++;
            lx->pos++;
            lx->col++;
        }
    }
}

/* ---------- 字符串 ---------- */

/* 追加一个字符串分段（用 Array 容器管理；OOM 由容器断言终止）。 */
static void sv_push(StringValue *sv, const StringSeg seg) {
    if (sv->segs == NULL) {
        sv->segs = arr_new(sizeof(StringSeg));
    }
    arr_push_v(sv->segs, &seg);
}

/* 解码含转义的字面量段：\n \r \t \\ \" \' \$；未知转义保留反斜杠。 */
static String decode_escapes(const char *src, size_t len) {
    char stack_buf[128];
    char *buf = (len < sizeof(stack_buf)) ? stack_buf : malloc(len + 1);
    assert(buf != NULL);
    size_t out = 0;
    for (size_t i = 0; i < len; i++) {
        char c = src[i];
        if (c == '\\' && i + 1 < len) {
            const char e = src[i + 1];
            switch (e) {
                case 'n': buf[out++] = '\n'; break;
                case 'r': buf[out++] = '\r'; break;
                case 't': buf[out++] = '\t'; break;
                case '\\': buf[out++] = '\\'; break;
                case '"': buf[out++] = '"'; break;
                case '\'': buf[out++] = '\''; break;
                case '$': buf[out++] = '$'; break;
                default: buf[out++] = '\\'; buf[out++] = e; break;
            }
            i++;
        } else {
            buf[out++] = c;
        }
    }
    const String result = str_new(buf, out);
    if (buf != stack_buf) free(buf);
    return result;
}

/* 追加字面量段：含转义则解码为 String，否则保存源视图。 */
static void push_literal(StringValue *sv, const char *start, size_t len) {
    if (len == 0) return;
    if (memchr(start, '\\', len) != NULL) {
        const String decoded = decode_escapes(start, len);
        sv_push(sv, (StringSeg){SEG_LITERAL_STR, {.str = decoded}});
    } else {
        sv_push(sv, (StringSeg){SEG_LITERAL_VIEW, {.view = {start, len}}});
    }
}

typedef enum { INTERP_OK, INTERP_UNTERMINATED, INTERP_NEWLINE } InterpStatus;

/* 扫描 "${" 之后的插值表达式，找到匹配的 "}"。
   正确处理 {} 嵌套、内部字符串（双/单引号）及其转义。 */
static InterpStatus scan_interp_expr(Lexer *lx, size_t *pp, const char **expr, size_t *expr_len) {
    size_t expr_start = *pp;
    size_t p = *pp;
    int depth = 1;
    char strq = 0;

    while (p < lx->len) {
        char c = lx->src[p];
        if (strq) {
            if (c == '\\') {
                p += 2;
                continue;
            }
            if (c == strq) strq = 0;
            p++;
            continue;
        }
        if (c == '"' || c == '\'') {
            strq = c;
            p++;
            continue;
        }
        if (c == '{') {
            depth++;
            p++;
            continue;
        }
        if (c == '}') {
            depth--;
            if (depth == 0) {
                *expr = lx->src + expr_start;
                *expr_len = p - expr_start;
                *pp = p + 1;
                return INTERP_OK;
            }
            p++;
            continue;
        }
        if (c == '\n' || c == '\r') return INTERP_NEWLINE;
        p++;
    }
    return INTERP_UNTERMINATED;
}

/* 普通字符串（双/单引号）：支持转义与 ${} 插值；不允许跨行。 */
static bool lex_scan_quoted(Lexer *lx, Token *tok, char quote) {
    size_t old = lx->pos;
    tok->start = lx->src + old;
    StringValue sv = {0};
    bool has_interp = false;
    bool has_escape = false;

    size_t p = old + 1;
    const char *lit = lx->src + p;   /* 当前字面量段起点 */
    size_t lit_len = 0;

    while (p < lx->len) {
        char c = lx->src[p];
        if (c == '\\') {
            /* `\`+换行：不允许跨行（与下方 newline 检查一致）。直接跳过下一字节会绕过
               '\n'/'\r' 检查 → 静默制造跨行字符串且行号不更新。 */
            if (p + 1 < lx->len && (lx->src[p + 1] == '\n' || lx->src[p + 1] == '\r')) {
                string_value_free(&sv);
                lex_error_at(lx, tok->line, tok->col, 1, "newline in string literal");
                return false;
            }
            has_escape = true;
            p += 2;
            lit_len += 2;
            continue;
        }
        if (c == '$' && p + 1 < lx->len && lx->src[p + 1] == '{') {
            has_interp = true;
            push_literal(&sv, lit, lit_len);
            p += 2;
            const char *expr = NULL;
            size_t expr_len = 0;
            InterpStatus st = scan_interp_expr(lx, &p, &expr, &expr_len);
            if (st != INTERP_OK) {
                string_value_free(&sv);
                if (st == INTERP_NEWLINE)
                    lex_error_at(lx, tok->line, tok->col, 1, "newline in string literal");
                else
                    lex_error_at(lx, tok->line, tok->col, 1, "unterminated interpolation in string");
                return false;
            }
            sv_push(&sv, (StringSeg){SEG_INTERP, {.view = {expr, expr_len}}});
            lit = lx->src + p;
            lit_len = 0;
            continue;
        }
        if (c == quote) {
            if (has_interp || has_escape) {
                push_literal(&sv, lit, lit_len);
                tok->value.str = sv;
            } else {
                tok->value.str = (StringValue){0};
            }
            tok->kind = TOK_STRING;
            tok->len = (p + 1) - old;
            lx->pos = old + tok->len;
            lx->col += (int)tok->len;
            return true;
        }
        if (c == '\n' || c == '\r') {
            string_value_free(&sv);
            lex_error_at(lx, tok->line, tok->col, 1, "newline in string literal");
            return false;
        }
        p++;
        lit_len++;
    }

    string_value_free(&sv);
    lex_error_at(lx, tok->line, tok->col, 1, "unterminated string literal");
    return false;
}

/* 原始字符串（"""..."""）：不转义、不插值，可跨行。 */
static bool lex_scan_raw(Lexer *lx, Token *tok) {
    const char *begin = lx->src + lx->pos;
    int start_line = lx->line, start_col = lx->col;
    lx->pos += 3;
    lx->col += 3;

    for (;;) {
        if (lx->pos >= lx->len) {
            lex_error_at(lx, start_line, start_col, 3, "unterminated string literal");
            return false;
        }
        if (lx->src[lx->pos] == '"' && lx->pos + 2 < lx->len &&
            lx->src[lx->pos + 1] == '"' && lx->src[lx->pos + 2] == '"') {
            lx->pos += 3;
            lx->col += 3;
            tok->kind = TOK_STRING;
            tok->start = begin;
            tok->len = (size_t)((lx->src + lx->pos) - begin);
            tok->value.str = (StringValue){0};
            return true;
        }
        if (lx->src[lx->pos] == '\n' || lx->src[lx->pos] == '\r') {
            lex_advance_byte(lx);
            continue;
        }
        lx->pos++;
        lx->col++;
    }
}

static bool lex_string(Lexer *lx, Token *tok) {
    if (lx->src[lx->pos] == '"' && lx->pos + 2 < lx->len &&
        lx->src[lx->pos + 1] == '"' && lx->src[lx->pos + 2] == '"') {
        return lex_scan_raw(lx, tok);
    }
    return lex_scan_quoted(lx, tok, lx->src[lx->pos]);
}

/* ---------- 对外 API ---------- */

Lexer *lexer_new_at(const char *source, size_t len, int line, int col) {
    Lexer *lx = calloc(1, sizeof(Lexer));
    if (!lx) return NULL;
    lx->src = source;
    lx->len = len;
    lx->line = line > 0 ? line : 1;
    lx->col = col > 0 ? col : 1;
    return lx;
}

Lexer *lexer_new(const char *source, size_t len) {
    return lexer_new_at(source, len, 1, 1);
}

void lexer_free(Lexer *lx) { free(lx); }

const LexError *lexer_error(const Lexer *lx) { return &lx->err; }

Token lexer_next(Lexer *lx) {
    Token tok;
    memset(&tok, 0, sizeof(tok));

    if (lx->errored) {
        tok.kind = TOK_EOF;
        tok.start = lx->src + lx->pos;
        tok.line = lx->line;
        tok.col = lx->col;
        return tok;
    }

    lex_skip_trivia(lx);
    if (lx->errored) {
        tok.kind = TOK_ERROR;
        tok.start = lx->src + lx->pos;
        tok.line = lx->err.line;
        tok.col = lx->err.col;
        return tok;
    }

    if (lx->pos >= lx->len) {
        tok.kind = TOK_EOF;
        tok.start = lx->src + lx->pos;
        tok.line = lx->line;
        tok.col = lx->col;
        return tok;
    }

    if (lx->newline_pending) {
        lx->newline_pending = false;
        tok.kind = TOK_NEWLINE;
        tok.start = lx->nl_start;
        tok.len = 0;
        tok.line = lx->nl_line;
        tok.col = lx->nl_col;
        return tok;
    }

    tok.start = lx->src + lx->pos;
    tok.line = lx->line;
    tok.col = lx->col;

    char c = lx->src[lx->pos];
    if (c >= '0' && c <= '9') {
        if (!lex_number(lx, &tok)) tok.kind = TOK_ERROR;
        return tok;
    }
    if (is_ident_start(c)) {
        lex_ident(lx, &tok);
        return tok;
    }
    if (c == '"' || c == '\'') {
        if (!lex_string(lx, &tok)) tok.kind = TOK_ERROR;
        return tok;
    }
    if (c == '$' && lx->pos + 1 < lx->len && is_digit(lx->src[lx->pos + 1])) {
        /* 闭包隐式参数 $0 / $1 / ...：词法上作标识符，语义层绑定为参数。 */
        size_t p = lx->pos + 1;
        while (p < lx->len && is_digit(lx->src[p])) p++;
        tok.kind = TOK_IDENT;
        tok.len = p - lx->pos;
        lx->pos = p;
        lx->col += (int)tok.len;
        return tok;
    }

    size_t mlen = 0;
    TokenKind k = token_operator_match(lx->src + lx->pos, lx->len - lx->pos, &mlen);
    if (k == TOK_EOF) {
        char msg[64];
        unsigned char uc = (unsigned char)lx->src[lx->pos];
        if (uc >= 0x20 && uc < 0x7f)
            snprintf(msg, sizeof(msg), "unexpected character '%c'", uc);
        else
            snprintf(msg, sizeof(msg), "unexpected character (0x%02X)", uc);
        lex_error_at(lx, tok.line, tok.col, 1, msg);
        tok.kind = TOK_ERROR;
        tok.len = 1;
        return tok;
    }
    tok.kind = k;
    tok.len = mlen;
    lx->pos += mlen;
    lx->col += (int)mlen;
    return tok;
}
