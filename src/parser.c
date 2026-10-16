#include "parser.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_ERRORS 20
#define MAX_PARSE_DEPTH 1000
/* 语句块嵌套上限（独立于表达式深度）：3000 层实测让编译器栈溢出，1500 层安全。 */
#define MAX_BLOCK_DEPTH 1500

struct Parser {
    Lexer *lex;
    Arena *arena;
    bool owns_arena;
    Token cur;
    Token next;
    int prev_line;   /* 上一个已消费 token 的行（尾随闭包判断同行动态） */
    bool allow_trailing_closure; /* 尾随闭包开关：控制流条件/iterable/match 表达式处关闭 */
    int depth;            /* 表达式递归深度（parse_expr；上限 MAX_PARSE_DEPTH） */
    int block_depth;      /* 语句块嵌套深度（parse_brace_body；上限 MAX_BLOCK_DEPTH）
                             与 depth 分开计数：共用会把"一块里写很多语句"也算成嵌套，
                             并在深处与表达式深度叠加而误报。 */
    int paren_depth;
    ParserError errors[MAX_ERRORS];
    size_t n_errors;
    bool fatal;
};

/* ---------- 动态数组（arena） ---------- */

#define ARR_PUSH(p, arr, n, cap, item)                                       \
    do {                                                                     \
        if ((n) == (cap)) {                                                  \
            size_t ncap = (cap) ? (cap) * 2 : 8;                             \
            void *na = arena_alloc((p)->arena, ncap * sizeof(*(arr)));       \
            if (arr) memcpy(na, (arr), (n) * sizeof(*(arr)));                \
            (arr) = (void *)na;                                              \
            (cap) = ncap;                                                    \
        }                                                                    \
        (arr)[(n)++] = (item);                                               \
    } while (0)

/* ---------- 错误 ---------- */

static void error_at(Parser *p, int line, int col, const char *msg) {
    if (p->n_errors >= MAX_ERRORS) {
        p->fatal = true;
        return;
    }
    ParserError *e = &p->errors[p->n_errors++];
    e->line = line;
    e->col = col;
    snprintf(e->message, sizeof(e->message), "%s", msg);
}

static void error_here(Parser *p, const char *msg) {
    error_at(p, p->cur.line, p->cur.col, msg);
}

/* ---------- token 管理 ---------- */

static void advance(Parser *p) {
    token_free(&p->cur);
    p->prev_line = p->cur.line;
    p->cur = p->next;
    p->next = lexer_next(p->lex);
    if (p->cur.kind == TOK_ERROR) {
        const LexError *e = lexer_error(p->lex);
        error_at(p, e->line, e->col, e->message);
        p->fatal = true;
    }
}

static bool check(Parser *p, TokenKind k) {
    return p->cur.kind == k;
}

static bool check_peek(Parser *p, TokenKind k) {
    return p->next.kind == k;
}

static bool match(Parser *p, TokenKind k) {
    if (check(p, k)) {
        advance(p);
        return true;
    }
    return false;
}

static bool expect(Parser *p, TokenKind k, const char *msg) {
    if (check(p, k)) {
        advance(p);
        return true;
    }
    error_here(p, msg);
    return false;
}

static void skip_newlines(Parser *p) {
    while (check(p, TOK_NEWLINE)) advance(p);
}

static void skip_semis(Parser *p) {
    skip_newlines(p);
    while (check(p, TOK_SEMICOLON)) {
        advance(p);
        skip_newlines(p);
    }
}

static const char *expect_ident(Parser *p, const char *msg) {
    if (check(p, TOK_IDENT)) {
        const char *s = arena_copy_str(p->arena, p->cur.start, p->cur.len);
        advance(p);
        return s;
    }
    if (msg) error_here(p, msg);
    return NULL;
}

static bool ident_is(const Parser *p, const char *text) {
    if (p->cur.kind != TOK_IDENT) return false;
    const size_t n = strlen(text);
    return p->cur.len == n && memcmp(p->cur.start, text, n) == 0;
}

static bool is_assign_op(TokenKind k) {
    switch (k) {
        case TOK_ASSIGN:
        case TOK_PLUS_ASSIGN:
        case TOK_MINUS_ASSIGN:
        case TOK_STAR_ASSIGN:
        case TOK_SLASH_ASSIGN:
        case TOK_PERCENT_ASSIGN:
        case TOK_STAR_STAR_ASSIGN:
        case TOK_SLASH_SLASH_ASSIGN:
            return true;
        default:
            return false;
    }
}

static bool is_stmt_start(TokenKind k) {
    switch (k) {
        case TOK_VAR:
        case TOK_LET:
        case TOK_IF:
        case TOK_WHILE:
        case TOK_FOR:
        case TOK_RETURN:
        case TOK_FUNC:
        case TOK_CLASS:
        case TOK_ENUM:
        case TOK_ERR:
        case TOK_IMPORT:
        case TOK_EXPORT:
        case TOK_BREAK:
        case TOK_CONTINUE:
        case TOK_DO:
        case TOK_LBRACE:
            return true;
        default:
            return false;
    }
}

/* 跳过一个语句到下一个语句边界（换行 / '}' / 语句起始关键字）。 */
static void synchronize(Parser *p) {
    while (!check(p, TOK_EOF)) {
        if (check(p, TOK_NEWLINE) || check(p, TOK_RBRACE) || check(p, TOK_SEMICOLON)) {
            return;
        }
        if (is_stmt_start(p->cur.kind)) return;
        advance(p);
    }
}

/* ---------- 运算符表（Pratt） ---------- */

typedef struct {
    TokenKind tok;
    int prec;
    bool right_assoc;
} BinOp;

static const BinOp k_binops[] = {
    {TOK_CATCH, 5, false},
    {TOK_QUESTION, 8, true},
    {TOK_QUESTION_QUESTION, 8, false}, /* ?? 空合并：a ?? b（a 非 null 取 a，否则 b） */
    {TOK_OR_OR, 10, false},
    {TOK_AND_AND, 20, false},
    {TOK_EQ, 30, false},
    {TOK_BANG_EQ, 30, false},
    {TOK_IS, 40, false},     /* is：类型判断（x is number / is Point / is Color.Red） */
    {TOK_LT, 40, false},
    {TOK_LE, 40, false},
    {TOK_GT, 40, false},
    {TOK_GE, 40, false},
    {TOK_DOT_DOT, 45, false},
    {TOK_PLUS, 50, false},
    {TOK_MINUS, 50, false},
    {TOK_STAR, 60, false},
    {TOK_SLASH, 60, false},
    {TOK_PERCENT, 60, false},
    {TOK_SLASH_SLASH, 60, false},
    {TOK_STAR_STAR, 70, true},
};

static const BinOp *lookup_binop(TokenKind k) {
    for (size_t i = 0; i < sizeof(k_binops) / sizeof(k_binops[0]); i++) {
        if (k_binops[i].tok == k) return &k_binops[i];
    }
    return NULL;
}

/* ---------- 前向声明 ---------- */

static AstNode *parse_expr(Parser *p, int min_prec);
static AstNode *parse_stmt(Parser *p);
static AstNode *parse_block(Parser *p);
static AstNode *parse_type_ref(Parser *p);
static AstNode *parse_if(Parser *p);
static AstNode *parse_while(Parser *p);
static AstNode *parse_for(Parser *p);
static AstNode *parse_func(Parser *p, bool require_name, bool is_static);
static CatchHandler parse_catch_handler(Parser *p);
static AstMatchArm parse_pattern(Parser *p, bool allow_bind);
static AstNode *parse_match(Parser *p);
static AstNode *parse_throw(Parser *p);
static AstNode *parse_enum(Parser *p);
static void parse_params(Parser *p, AstParam **out_params, size_t *out_n, bool *out_varargs);

/* ---------- 表达式 ---------- */

static AstNode *parse_string(Parser *p);
static AstNode *parse_interp_expr(Parser *p, const char *text, size_t len);
static AstNode *parse_call_tail(Parser *p, AstNode *callee);
static AstNode *parse_index_tail(Parser *p, AstNode *obj);
static AstNode *parse_member_tail(Parser *p, AstNode *obj);
static AstNode *parse_try_call(Parser *p, AstNode *left);       /* X.try m() / X.try? m() */
static AstNode *parse_optional_chain(Parser *p, AstNode *left); /* a?.b / a?.b(...) */
static AstNode *parse_paren_or_lambda(Parser *p);
static AstNode *parse_array(Parser *p);
static AstNode *parse_map_or_closure(Parser *p);
static AstNode *parse_closure(Parser *p, int line, int col);
static int scan_implicit_arg_max(const AstNode *n);
static AstNode *parse_trailing_closure(Parser *p, AstNode *callee);
static bool is_callable_trailing(const AstNode *n);
static AstNode *parse_expr_no_trailing(Parser *p);
static AstNode *parse_expr_sub(Parser *p);
static AstNode *parse_block_expr(Parser *p);
static void mark_no_value_stmt(Parser *p, AstNode *s);
static AstNode *parse_prop_body(Parser *p, int line, int col);
static AstNode *make_getter_prop(Parser *p, int line, int col, const char *name, AstNode *body,
                                 bool is_static);
static AstNode *parse_func_literal(Parser *p);

static AstNode *parse_prefix(Parser *p) {
    switch (p->cur.kind) {
        case TOK_NUMBER: {
            AstNode *n = ast_new(p->arena, AST_NUMBER, p->cur.line, p->cur.col);
            n->as.number.text = arena_copy_str(p->arena, p->cur.start, p->cur.len);
            n->as.number.len = p->cur.len;
            advance(p);
            return n;
        }
        case TOK_STRING:
            return parse_string(p);
        case TOK_IDENT: {
            AstNode *n = ast_new(p->arena, AST_IDENT, p->cur.line, p->cur.col);
            n->as.ident.name = arena_copy_str(p->arena, p->cur.start, p->cur.len);
            advance(p);
            return n;
        }
        case TOK_COROUTINE: { /* coroutine 是关键字保留字（协程命名空间），作标识符用 */
            AstNode *n = ast_new(p->arena, AST_IDENT, p->cur.line, p->cur.col);
            n->as.ident.name = arena_copy_str(p->arena, "coroutine", 9);
            advance(p);
            return n;
        }
        case TOK_DOLLAR_LBRACE:
            return parse_block_expr(p); /* ${ ... } 块表达式 */
        case TOK_NULL:
        case TOK_TRUE:
        case TOK_FALSE:
        case TOK_UNINITIALIZED: {
            AstKind k;
            if (p->cur.kind == TOK_NULL) k = AST_NULL;
            else if (p->cur.kind == TOK_TRUE) k = AST_TRUE;
            else if (p->cur.kind == TOK_FALSE) k = AST_FALSE;
            else k = AST_UNINITIALIZED;
            AstNode *n = ast_new(p->arena, k, p->cur.line, p->cur.col);
            advance(p);
            return n;
        }
        case TOK_SUPER: {
            AstNode *n = ast_new(p->arena, AST_IDENT, p->cur.line, p->cur.col);
            n->as.ident.name = arena_copy_str(p->arena, "super", 5);
            advance(p);
            return n;
        }
        case TOK_DOT_DOT_DOT: {
            AstNode *n = ast_new(p->arena, AST_VARARGS, p->cur.line, p->cur.col);
            advance(p);
            return n;
        }
        case TOK_ERR: {
            int line = p->cur.line, col = p->cur.col;
            advance(p);
            if (!expect(p, TOK_LPAREN, "expected '(' after error")) return NULL;
            skip_newlines(p);
            AstNode *msg = parse_expr(p, 0);
            skip_newlines(p);
            if (!expect(p, TOK_RPAREN, "expected ')' after error message")) return NULL;
            AstNode *n = ast_new(p->arena, AST_ERROR_VAL, line, col);
            n->as.error_val.msg = msg;
            return n;
        }
        case TOK_MATCH:
            return parse_match(p);
        case TOK_MINUS:
        case TOK_PLUS:
        case TOK_BANG: {
            int op = p->cur.kind;
            int line = p->cur.line, col = p->cur.col;
            advance(p);
            skip_newlines(p);
            AstNode *operand = parse_expr(p, 65);
            if (!operand) return NULL;
            AstNode *n = ast_new(p->arena, AST_UNARY, line, col);
            n->as.unary.op = op;
            n->as.unary.operand = operand;
            return n;
        }
        case TOK_TRY:
        case TOK_TRY_QUESTION:
        case TOK_TRY_BANG: {
            int kind = p->cur.kind;
            int line = p->cur.line, col = p->cur.col;
            advance(p);
            skip_newlines(p);
            if (check(p, TOK_LBRACE)) {
                /* try { stmts } / try! { stmts }：语句块（块内错误上抛/终止）。仅 try/try!。 */
                if (kind == TOK_TRY_QUESTION) {
                    error_at(p, line, col, "try? block is not supported (only try and try! blocks)");
                    return NULL;
                }
                AstNode *body = parse_block(p);
                if (!body) return NULL;
                AstNode *n = ast_new(p->arena, AST_TRY_BLOCK, line, col);
                n->as.try_block.kind = kind;
                n->as.try_block.body = body;
                return n;
            }
            AstNode *e = parse_expr(p, 6);
            if (!e) return NULL;
            AstNode *n = ast_new(p->arena, AST_TRY, line, col);
            n->as.try_.kind = kind;
            n->as.try_.expr = e;
            return n;
        }
        case TOK_THIS_FUNC: {
            AstNode *n = ast_new(p->arena, AST_THIS_FUNC, p->cur.line, p->cur.col);
            advance(p);
            return n;
        }
        case TOK_YIELD: {
            const int line = p->cur.line, col = p->cur.col;
            advance(p);
            AstNode *n = ast_new(p->arena, AST_YIELD, line, col);
            n->as.yield_.expr = NULL;
            if (!check(p, TOK_NEWLINE) && !check(p, TOK_EOF) && !check(p, TOK_RPAREN) &&
                !check(p, TOK_RBRACE) && !check(p, TOK_COMMA) && !check(p, TOK_SEMICOLON)) {
                skip_newlines(p);
                n->as.yield_.expr = parse_expr(p, 0);
            }
            return n;
        }
        case TOK_LPAREN:
            return parse_paren_or_lambda(p);
        case TOK_LBRACKET:
            return parse_array(p);
        case TOK_LBRACE:
            return parse_map_or_closure(p);
        case TOK_IF:
            return parse_if(p);
        case TOK_WHILE:
            return parse_while(p);
        case TOK_FOR:
            return parse_for(p);
        case TOK_FUNC:
            return parse_func_literal(p);
        default:
            error_here(p, "unexpected token in expression");
            advance(p); /* 消耗坏 token，保证循环推进 */
            return NULL;
    }
}

/* 字符串：把 token 的 StringValue 分段转为 AST parts。 */
static AstNode *parse_string(Parser *p) {
    Token *tok = &p->cur;
    const int line = tok->line, col = tok->col;
    AstStrPart *parts = NULL;
    size_t n_parts = 0, cap = 0;

    if (tok->value.str.segs == NULL) {
        const char *inner;
        size_t inner_len;
        if (tok->len >= 6 && tok->start[0] == '"' && tok->start[1] == '"' && tok->start[2] == '"') {
            inner = tok->start + 3;
            inner_len = tok->len - 6;
        } else {
            inner = tok->start + 1;
            inner_len = tok->len - 2;
        }
        const char *text = arena_copy_str(p->arena, inner, inner_len);
        AstStrPart part = {true, text, inner_len, NULL};
        ARR_PUSH(p, parts, n_parts, cap, part);
    } else {
        const size_t m = arr_size(tok->value.str.segs);
        for (size_t i = 0; i < m; i++) {
            const StringSeg *seg = (const StringSeg *)arr_at(tok->value.str.segs, i);
            AstStrPart part;
            if (seg->kind == SEG_INTERP) {
                part.is_literal = false;
                part.expr = parse_interp_expr(p, seg->data.view.start, seg->data.view.len);
                part.text = NULL;
                part.len = 0;
            } else if (seg->kind == SEG_LITERAL_STR) {
                const char *text = str_data(&seg->data.str);
                const size_t tlen = str_len(&seg->data.str);
                part.is_literal = true;
                part.text = arena_copy_str(p->arena, text, tlen);
                part.len = tlen;
                part.expr = NULL;
            } else { /* SEG_LITERAL_VIEW */
                part.is_literal = true;
                part.text = arena_copy_str(p->arena, seg->data.view.start, seg->data.view.len);
                part.len = seg->data.view.len;
                part.expr = NULL;
            }
            ARR_PUSH(p, parts, n_parts, cap, part);
        }
    }

    advance(p); /* 释放 STRING token（含其 StringValue） */
    AstNode *n = ast_new(p->arena, AST_STRING, line, col);
    n->as.str.parts = parts;
    n->as.str.n_parts = n_parts;
    return n;
}

/* 用子 parser 解析插值表达式（共享主 arena）。
   子词法器从字符串 token 的绝对位置起算，使内部错误与 AST 行列落到源码真实位置。 */
static AstNode *parse_interp_expr(Parser *p, const char *text, size_t len) {
    const int abs_line = p->cur.line;
    const int abs_col = p->cur.col + (int)(text - p->cur.start);
    Lexer *lex = lexer_new_at(text, len, abs_line, abs_col);
    if (!lex) {
        error_here(p, "out of memory");
        return NULL;
    }
    Parser *sub = parser_new_shared(lex, p->arena);
    /* 继承父 parser 的递归深度：子 parser 的 depth 从 0 起会让 `"${"${...}"}"`
       每层重置护栏（实测 4000 层嵌套插值直接栈溢出打死编译器）。 */
    sub->depth = p->depth;
    AstNode *e = parse_expr(sub, 0);
    if (!check(sub, TOK_EOF) && sub->cur.kind != TOK_ERROR) {
        error_at(p, sub->cur.line, sub->cur.col, "unexpected token after interpolation expression");
    }
    for (size_t i = 0; i < sub->n_errors && p->n_errors < MAX_ERRORS; i++) {
        p->errors[p->n_errors++] = sub->errors[i];
    }
    if (sub->n_errors > 0 && p->n_errors >= MAX_ERRORS) {
        p->fatal = true;
    }
    parser_free(sub);
    lexer_free(lex);
    return e;
}

static AstNode *parse_call_tail(Parser *p, AstNode *callee) {
    const int line = callee->line, col = callee->col;
    advance(p); /* ( */
    p->paren_depth++;
    skip_newlines(p);
    AstArg *args = NULL;
    size_t n_args = 0, cap = 0;
    while (!check(p, TOK_RPAREN) && !check(p, TOK_EOF)) {
        AstArg arg = {0};
        if (check(p, TOK_IDENT) && check_peek(p, TOK_ASSIGN)) {
            arg.named = true;
            arg.name = arena_copy_str(p->arena, p->cur.start, p->cur.len);
            advance(p);
            advance(p); /* = */
            arg.expr = parse_expr_sub(p);
        } else {
            arg.expr = parse_expr_sub(p);
        }
        if (!arg.expr) break;
        ARR_PUSH(p, args, n_args, cap, arg);
        skip_newlines(p);
        if (match(p, TOK_COMMA)) {
            skip_newlines(p);
            continue;
        }
        break;
    }
    skip_newlines(p);
    expect(p, TOK_RPAREN, "expected ')' after arguments");
    p->paren_depth--;
    AstNode *n = ast_new(p->arena, AST_CALL, line, col);
    n->as.call.callee = callee;
    n->as.call.args = args;
    n->as.call.n_args = n_args;
    return n;
}

static AstNode *parse_index_tail(Parser *p, AstNode *obj) {
    const int line = obj->line, col = obj->col;
    advance(p); /* [ */
    p->paren_depth++;
    skip_newlines(p);
    AstNode *idx = parse_expr_sub(p);
    skip_newlines(p);
    expect(p, TOK_RBRACKET, "expected ']' after index");
    p->paren_depth--;
    AstNode *n = ast_new(p->arena, AST_INDEX, line, col);
    n->as.index.obj = obj;
    n->as.index.index = idx;
    return n;
}

static AstNode *parse_member_tail(Parser *p, AstNode *obj) {
    const int line = obj->line, col = obj->col;
    advance(p); /* . */
    if (!check(p, TOK_IDENT)) {
        error_here(p, "expected member name after '.'");
        return NULL;
    }
    const char *name = arena_copy_str(p->arena, p->cur.start, p->cur.len);
    advance(p);
    AstNode *n = ast_new(p->arena, AST_MEMBER, line, col);
    n->as.member.obj = obj;
    n->as.member.name = name;
    return n;
}

/* `X.try method(args)` / `X.try? method(args)`：在 X 上调用 method 并加 try 保护。
   链式调用 `obj.try m1().try? m2()` 的语法糖 = `try X.m1()` 后 `try? (结果).m2()`。
   cur 是 '.'，next 是 try/try?。构造 AST_TRY{kind, expr=AST_CALL{callee=AST_MEMBER{X, method}}}。 */
static AstNode *parse_try_call(Parser *p, AstNode *left) {
    const TokenKind kind = p->next.kind; /* TOK_TRY or TOK_TRY_QUESTION */
    const int line = left->line, col = left->col;
    advance(p); /* . */
    advance(p); /* try / try? */
    skip_newlines(p);
    const char *name = expect_ident(p, "expected method name after '.try'");
    if (!name) return left;
    if (!check(p, TOK_LPAREN)) {
        error_here(p, "expected '(' after method name in '.try' call");
        return left;
    }
    AstNode *member = ast_new(p->arena, AST_MEMBER, line, col);
    member->as.member.obj = left;
    member->as.member.name = name;
    AstNode *call = parse_call_tail(p, member);
    AstNode *n = ast_new(p->arena, AST_TRY, line, col);
    n->as.try_.kind = kind;
    n->as.try_.expr = call;
    return n;
}

/* `a?.b` / `a?.b(args)`：a 为 null 得 null（不访问 .b），否则 a.b 或 a.b(args)。cur 是 '?.'。 */
static AstNode *parse_optional_chain(Parser *p, AstNode *left) {
    const int line = left->line, col = left->col;
    advance(p); /* ?. */
    skip_newlines(p);
    const char *name = expect_ident(p, "expected member name after '?.'");
    if (!name) return left;
    AstNode *n = ast_new(p->arena, AST_OPTIONAL, line, col);
    n->as.optional.obj = left;
    n->as.optional.name = name;
    n->as.optional.args = NULL;
    n->as.optional.n_args = 0;
    n->as.optional.is_call = false;
    if (check(p, TOK_LPAREN)) {
        AstNode *member = ast_new(p->arena, AST_MEMBER, line, col);
        member->as.member.obj = left;
        member->as.member.name = name;
        AstNode *call = parse_call_tail(p, member);
        n->as.optional.args = call->as.call.args;
        n->as.optional.n_args = call->as.call.n_args;
        n->as.optional.is_call = true;
    }
    return n;
}

static void parse_lambda_body(Parser *p, AstNode **out_body) {
    skip_newlines(p);
    if (check(p, TOK_LBRACE)) {
        *out_body = parse_block(p);
    } else {
        *out_body = parse_expr_sub(p);
    }
}

static AstNode *parse_paren_or_lambda(Parser *p) {
    const int line = p->cur.line, col = p->cur.col;
    advance(p); /* ( */
    p->paren_depth++;
    skip_newlines(p);

    if (check(p, TOK_RPAREN)) {
        advance(p);
        p->paren_depth--;
        skip_newlines(p);
        const bool is_throws = match(p, TOK_THROWS);
        if (match(p, TOK_ARROW)) {
            AstNode *body = NULL;
            parse_lambda_body(p, &body);
            AstNode *n = ast_new(p->arena, AST_LAMBDA, line, col);
            n->as.lambda.params = NULL;
            n->as.lambda.n_params = 0;
            n->as.lambda.has_varargs = false;
            n->as.lambda.is_throws = is_throws;
            n->as.lambda.body = body;
            return n;
        }
        error_at(p, line, col, "unexpected ')' in expression");
        return NULL;
    }

    AstNode **items = NULL;
    size_t n_items = 0, cap = 0;
    for (;;) {
        AstNode *e = parse_expr_sub(p);
        if (!e) break;
        ARR_PUSH(p, items, n_items, cap, e);
        skip_newlines(p);
        if (match(p, TOK_COMMA)) {
            skip_newlines(p);
            continue;
        }
        break;
    }
    skip_newlines(p);
    expect(p, TOK_RPAREN, "expected ')'");
    p->paren_depth--;
    skip_newlines(p);
    const bool is_throws = match(p, TOK_THROWS);

    if (match(p, TOK_ARROW)) {
        AstParam *params = NULL;
        size_t n_params = 0, cap_p = 0;
        bool has_varargs = false;
        for (size_t i = 0; i < n_items; i++) {
            AstNode *item = items[i];
            AstParam param = {0};
            if (item->kind == AST_IDENT) {
                param.name = item->as.ident.name;
            } else if (item->kind == AST_VARARGS) {
                has_varargs = true;
                continue;
            } else {
                error_at(p, item->line, item->col, "lambda parameter must be an identifier");
                continue;
            }
            ARR_PUSH(p, params, n_params, cap_p, param);
        }
        AstNode *body = NULL;
        parse_lambda_body(p, &body);
        AstNode *n = ast_new(p->arena, AST_LAMBDA, line, col);
        n->as.lambda.params = params;
        n->as.lambda.n_params = n_params;
        n->as.lambda.has_varargs = has_varargs;
        n->as.lambda.is_throws = is_throws;
        n->as.lambda.body = body;
        return n;
    }

    if (n_items != 1) {
        error_at(p, line, col, "expected single expression in parentheses");
        return NULL;
    }
    return items[0];
}

static AstNode *parse_array(Parser *p) {
    const int line = p->cur.line, col = p->cur.col;
    advance(p); /* [ */
    p->paren_depth++;
    skip_newlines(p);
    AstNode **elems = NULL;
    size_t n = 0, cap = 0;
    while (!check(p, TOK_RBRACKET) && !check(p, TOK_EOF)) {
        AstNode *e = parse_expr_sub(p);
        if (!e) break;
        ARR_PUSH(p, elems, n, cap, e);
        skip_newlines(p);
        if (match(p, TOK_COMMA)) {
            skip_newlines(p);
            continue;
        }
        break;
    }
    skip_newlines(p);
    expect(p, TOK_RBRACKET, "expected ']' after array");
    p->paren_depth--;
    AstNode *node = ast_new(p->arena, AST_ARRAY, line, col);
    node->as.array.elems = elems;
    node->as.array.n = n;
    return node;
}

static AstNode *parse_map(Parser *p, int line, int col) {
    /* cur 是 '{' 之后、第一个 key 处 */
    AstMapEntry *entries = NULL;
    size_t n = 0, cap = 0;
    while (!check(p, TOK_RBRACE) && !check(p, TOK_EOF)) {
        AstNode *key = parse_expr_sub(p);
        if (!key) break;
        skip_newlines(p);
        expect(p, TOK_COLON, "expected ':' after map key");
        skip_newlines(p);
        AstNode *value = parse_expr_sub(p);
        if (!value) break;
        AstMapEntry entry = {key, value};
        ARR_PUSH(p, entries, n, cap, entry);
        skip_newlines(p);
        if (match(p, TOK_COMMA)) {
            skip_newlines(p);
            continue;
        }
        break;
    }
    skip_newlines(p);
    expect(p, TOK_RBRACE, "expected '}' after map");
    p->paren_depth--;
    AstNode *node = ast_new(p->arena, AST_MAP, line, col);
    node->as.map.entries = entries;
    node->as.map.n = n;
    return node;
}

static bool is_implicit_arg_name(const char *name) {
    if (!name || name[0] != '$') return false;
    for (const char *p = name + 1; *p; p++) {
        if (*p < '0' || *p > '9') return false;
    }
    return name[1] != '\0';
}

/* 扫描树中最大 $N（隐式参数下标）。不深入独立函数作用域（lambda/func/defer）。 */
static int scan_implicit_arg_max(const AstNode *n) {
    if (!n) return -1;
    switch (n->kind) {
        case AST_IDENT: {
            if (!is_implicit_arg_name(n->as.ident.name)) return -1;
            /* 有界解析（atoi 对超长数字串是 UB，且巨额 $N 会让 lambda 推导出天文参数数 →
               arena_alloc 巨额断言中止）。封顶 255：病态 $N 走下游"寄存器过多"编译报错。 */
            int v = 0;
            for (const char *p = n->as.ident.name + 1; *p; p++) {
                v = v * 10 + (*p - '0');
                if (v > 255) return 255;
            }
            return v;
        }
        case AST_LAMBDA:
        case AST_FUNC_DECL:
        case AST_DEFER:
            return -1;
        case AST_ERROR_VAL: return scan_implicit_arg_max(n->as.error_val.msg);
        case AST_ARRAY: {
            int m = -1;
            for (size_t i = 0; i < n->as.array.n; i++) {
                int v = scan_implicit_arg_max(n->as.array.elems[i]);
                if (v > m) m = v;
            }
            return m;
        }
        case AST_MAP: {
            int m = -1;
            for (size_t i = 0; i < n->as.map.n; i++) {
                int v = scan_implicit_arg_max(n->as.map.entries[i].key);
                if (v > m) m = v;
                v = scan_implicit_arg_max(n->as.map.entries[i].value);
                if (v > m) m = v;
            }
            return m;
        }
        case AST_UNARY: return scan_implicit_arg_max(n->as.unary.operand);
        case AST_BINARY: {
            int a = scan_implicit_arg_max(n->as.binary.left);
            int b = scan_implicit_arg_max(n->as.binary.right);
            return a > b ? a : b;
        }
        case AST_TERNARY: {
            int a = scan_implicit_arg_max(n->as.ternary.cond);
            int b = scan_implicit_arg_max(n->as.ternary.then_);
            int c = scan_implicit_arg_max(n->as.ternary.else_);
            int m = a > b ? a : b;
            return c > m ? c : m;
        }
        case AST_CALL: {
            int m = scan_implicit_arg_max(n->as.call.callee);
            for (size_t i = 0; i < n->as.call.n_args; i++) {
                int v = scan_implicit_arg_max(n->as.call.args[i].expr);
                if (v > m) m = v;
            }
            return m;
        }
        case AST_INDEX: {
            int a = scan_implicit_arg_max(n->as.index.obj);
            int b = scan_implicit_arg_max(n->as.index.index);
            return a > b ? a : b;
        }
        case AST_MEMBER: return scan_implicit_arg_max(n->as.member.obj);
        case AST_YIELD: return scan_implicit_arg_max(n->as.yield_.expr);
        case AST_TRY: return scan_implicit_arg_max(n->as.try_.expr);
        case AST_TRY_BLOCK: return scan_implicit_arg_max(n->as.try_block.body);
        case AST_THROW: return scan_implicit_arg_max(n->as.throw_.expr);
        case AST_OPTIONAL: {
            int m = scan_implicit_arg_max(n->as.optional.obj);
            for (size_t i = 0; i < n->as.optional.n_args; i++) {
                int v = scan_implicit_arg_max(n->as.optional.args[i].expr);
                if (v > m) m = v;
            }
            return m;
        }
        case AST_DO_CATCH: {
            int m = scan_implicit_arg_max(n->as.do_catch.body);
            const CatchHandler *h = &n->as.do_catch.handler;
            if (h->kind == CATCH_FALLBACK || h->kind == CATCH_BLOCK) {
                int v = scan_implicit_arg_max(h->fallback);
                if (v > m) m = v;
            }
            for (size_t i = 0; i < h->n_arms; i++) {
                int v = scan_implicit_arg_max(h->arms[i].value);
                if (v > m) m = v;
            }
            return m;
        }
        case AST_CATCH: {
            int m = scan_implicit_arg_max(n->as.catch_.expr);
            const CatchHandler *h = &n->as.catch_.handler;
            if (h->kind == CATCH_FALLBACK || h->kind == CATCH_BLOCK) {
                int v = scan_implicit_arg_max(h->fallback);
                if (v > m) m = v;
            }
            for (size_t i = 0; i < h->n_arms; i++) {
                int v = scan_implicit_arg_max(h->arms[i].value);
                if (v > m) m = v;
            }
            return m;
        }
        case AST_MATCH: {
            int m = scan_implicit_arg_max(n->as.match_.expr);
            for (size_t i = 0; i < n->as.match_.n_arms; i++) {
                int v = scan_implicit_arg_max(n->as.match_.arms[i].value);
                if (v > m) m = v;
            }
            return m;
        }
        case AST_VAR_DECL: {
            int m = -1;
            for (size_t i = 0; i < n->as.var_decl.n_values; i++) {
                int v = scan_implicit_arg_max(n->as.var_decl.values[i]);
                if (v > m) m = v;
            }
            return m;
        }
        case AST_ASSIGN: {
            int m = -1;
            for (size_t i = 0; i < n->as.assign.n; i++) {
                int v = scan_implicit_arg_max(n->as.assign.targets[i]);
                if (v > m) m = v;
            }
            for (size_t i = 0; i < n->as.assign.n_values; i++) {
                int v = scan_implicit_arg_max(n->as.assign.values[i]);
                if (v > m) m = v;
            }
            return m;
        }
        case AST_EXPR_STMT: return scan_implicit_arg_max(n->as.expr_stmt.expr);
        case AST_IF: {
            int m = scan_implicit_arg_max(n->as.if_.cond);
            int v = scan_implicit_arg_max(n->as.if_.then_);
            if (v > m) m = v;
            for (size_t i = 0; i < n->as.if_.n_elif; i++) {
                v = scan_implicit_arg_max(n->as.if_.elif_conds[i]);
                if (v > m) m = v;
                v = scan_implicit_arg_max(n->as.if_.elif_bodies[i]);
                if (v > m) m = v;
            }
            v = scan_implicit_arg_max(n->as.if_.else_);
            return v > m ? v : m;
        }
        case AST_WHILE: {
            int a = scan_implicit_arg_max(n->as.while_.cond);
            int b = scan_implicit_arg_max(n->as.while_.body);
            return a > b ? a : b;
        }
        case AST_FOR_IN: {
            int m = scan_implicit_arg_max(n->as.for_in.iterable);
            int v = scan_implicit_arg_max(n->as.for_in.step);
            if (v > m) m = v;
            v = scan_implicit_arg_max(n->as.for_in.body);
            return v > m ? v : m;
        }
        case AST_RETURN: {
            int m = -1;
            for (size_t i = 0; i < n->as.return_.n; i++) {
                int v = scan_implicit_arg_max(n->as.return_.values[i]);
                if (v > m) m = v;
            }
            return m;
        }
        case AST_BLOCK:
        case AST_BLOCK_EXPR: {
            int m = -1;
            for (size_t i = 0; i < n->as.block.n; i++) {
                int v = scan_implicit_arg_max(n->as.block.stmts[i]);
                if (v > m) m = v;
            }
            return m;
        }
        default:
            return -1;
    }
}

/* 树中是否含 yield（不深入嵌套 func/lambda/defer——它们各自独立协程函数）。 */
static bool parser_ast_has_yield(const AstNode *n) {
    if (!n) return false;
    switch (n->kind) {
        case AST_YIELD: return true;
        case AST_TRY_BLOCK: return parser_ast_has_yield(n->as.try_block.body);
        case AST_OPTIONAL:
            if (parser_ast_has_yield(n->as.optional.obj)) return true;
            for (size_t i = 0; i < n->as.optional.n_args; i++)
                if (parser_ast_has_yield(n->as.optional.args[i].expr)) return true;
            return false;
        case AST_DO_CATCH: {
            if (parser_ast_has_yield(n->as.do_catch.body)) return true;
            const CatchHandler *h = &n->as.do_catch.handler;
            if (h->kind == CATCH_FALLBACK || h->kind == CATCH_BLOCK)
                return parser_ast_has_yield(h->fallback);
            for (size_t i = 0; i < h->n_arms; i++)
                if (parser_ast_has_yield(h->arms[i].value)) return true;
            return false;
        }
        case AST_LAMBDA:
        case AST_FUNC_DECL:
        case AST_DEFER:
        case AST_PROP_DECL:
            return false;
        case AST_BLOCK:
        case AST_BLOCK_EXPR:
            for (size_t i = 0; i < n->as.block.n; i++)
                if (parser_ast_has_yield(n->as.block.stmts[i])) return true;
            return false;
        case AST_EXPR_STMT: return parser_ast_has_yield(n->as.expr_stmt.expr);
        case AST_RETURN:
            for (size_t i = 0; i < n->as.return_.n; i++)
                if (parser_ast_has_yield(n->as.return_.values[i])) return true;
            return false;
        case AST_UNARY: return parser_ast_has_yield(n->as.unary.operand);
        case AST_BINARY:
            return parser_ast_has_yield(n->as.binary.left) || parser_ast_has_yield(n->as.binary.right);
        case AST_TERNARY:
            return parser_ast_has_yield(n->as.ternary.cond) || parser_ast_has_yield(n->as.ternary.then_) ||
                   parser_ast_has_yield(n->as.ternary.else_);
        case AST_CALL:
            if (parser_ast_has_yield(n->as.call.callee)) return true;
            for (size_t i = 0; i < n->as.call.n_args; i++)
                if (parser_ast_has_yield(n->as.call.args[i].expr)) return true;
            return false;
        case AST_INDEX:
            return parser_ast_has_yield(n->as.index.obj) || parser_ast_has_yield(n->as.index.index);
        case AST_MEMBER: return parser_ast_has_yield(n->as.member.obj);
        case AST_ARRAY:
            for (size_t i = 0; i < n->as.array.n; i++)
                if (parser_ast_has_yield(n->as.array.elems[i])) return true;
            return false;
        case AST_MAP:
            for (size_t i = 0; i < n->as.map.n; i++) {
                if (parser_ast_has_yield(n->as.map.entries[i].key)) return true;
                if (parser_ast_has_yield(n->as.map.entries[i].value)) return true;
            }
            return false;
        case AST_IF:
            if (parser_ast_has_yield(n->as.if_.cond)) return true;
            if (n->as.if_.then_ && parser_ast_has_yield(n->as.if_.then_)) return true;
            for (size_t i = 0; i < n->as.if_.n_elif; i++) {
                if (parser_ast_has_yield(n->as.if_.elif_conds[i])) return true;
                if (n->as.if_.elif_bodies[i] && parser_ast_has_yield(n->as.if_.elif_bodies[i])) return true;
            }
            return n->as.if_.else_ && parser_ast_has_yield(n->as.if_.else_);
        case AST_WHILE:
            return parser_ast_has_yield(n->as.while_.cond) || parser_ast_has_yield(n->as.while_.body);
        case AST_FOR_IN:
            if (parser_ast_has_yield(n->as.for_in.iterable)) return true;
            if (n->as.for_in.step && parser_ast_has_yield(n->as.for_in.step)) return true;
            return parser_ast_has_yield(n->as.for_in.body);
        case AST_VAR_DECL:
            for (size_t i = 0; i < n->as.var_decl.n_values; i++)
                if (parser_ast_has_yield(n->as.var_decl.values[i])) return true;
            return false;
        case AST_ASSIGN:
            for (size_t i = 0; i < n->as.assign.n; i++)
                if (parser_ast_has_yield(n->as.assign.targets[i])) return true;
            for (size_t i = 0; i < n->as.assign.n_values; i++)
                if (parser_ast_has_yield(n->as.assign.values[i])) return true;
            return false;
        case AST_THROW: return parser_ast_has_yield(n->as.throw_.expr);
        default:
            return false;
    }
}

/* 闭包字面量 { ... }：Swift 风格 lambda。隐式参数 $0/$1/... 由体内最大引用推导；
   体是函数体语义（用 return 返回值）。cur 是 '{' 之后、第一个语句处。 */
static AstNode *parse_closure(Parser *p, int line, int col) {
    AstNode **stmts = NULL;
    size_t n = 0, cap = 0;
    while (!check(p, TOK_RBRACE) && !check(p, TOK_EOF)) {
        const char *stmt_start = p->cur.start;
        AstNode *s = parse_stmt(p);
        mark_no_value_stmt(p, s);
        if (s) ARR_PUSH(p, stmts, n, cap, s);
        if (p->fatal) break;
        skip_semis(p);
        if (check(p, TOK_RBRACE) || check(p, TOK_EOF)) break;
        if (p->cur.start == stmt_start &&
            !is_stmt_start(p->cur.kind) && !check(p, TOK_NEWLINE) && !check(p, TOK_SEMICOLON)) {
            synchronize(p);
        }
    }
    skip_newlines(p);
    expect(p, TOK_RBRACE, "expected '}' after closure body");
    p->paren_depth--;

    AstNode *block = ast_new(p->arena, AST_BLOCK, line, col);
    block->as.block.stmts = stmts;
    block->as.block.n = n;

    /* 单表达式闭包体可省略 return（Swift 风格）：唯一一条非 `;` 结尾的表达式语句 → 体即该表达式。
       `{ $0 * 2 }` / 尾随闭包 `arr.map { $0 * 2 }` 隐式返回；`{ f(); }`（`;` 结尾的调用）不省略。 */
    AstNode *body = block;
    if (n == 1 && stmts[0]->kind == AST_EXPR_STMT && !stmts[0]->as.expr_stmt.no_value) {
        body = stmts[0]->as.expr_stmt.expr;
    }

    AstParam *params = NULL;
    size_t n_params = 0;
    const int max_arg = scan_implicit_arg_max(body);
    if (max_arg >= 0) {
        n_params = (size_t)max_arg + 1;
        params = arena_alloc(p->arena, n_params * sizeof(AstParam));
        for (size_t i = 0; i < n_params; i++) {
            char namebuf[16];
            snprintf(namebuf, sizeof(namebuf), "$%zu", i);
            params[i].name = arena_copy_str(p->arena, namebuf, strlen(namebuf));
            params[i].is_self = false;
            params[i].has_default = false;
            params[i].default_expr = NULL;
        }
    }

    AstNode *node = ast_new(p->arena, AST_LAMBDA, line, col);
    node->as.lambda.params = params;
    node->as.lambda.n_params = n_params;
    node->as.lambda.has_varargs = false;
    node->as.lambda.is_throws = false;
    node->as.lambda.body = body;
    return node;
}

static AstNode *parse_map_or_closure(Parser *p) {
    const int line = p->cur.line, col = p->cur.col;
    advance(p); /* { */
    p->paren_depth++;
    skip_newlines(p);

    if (check(p, TOK_RBRACE)) {
        return parse_map(p, line, col); /* 空 {} → 空 map */
    }
    if ((check(p, TOK_STRING) || check(p, TOK_IDENT) || check(p, TOK_NUMBER)) &&
        check_peek(p, TOK_COLON)) {
        return parse_map(p, line, col);
    }

    return parse_closure(p, line, col);
}

/* 尾随闭包：`obj.m { ... }` / `f(a) { ... }` / `f { ... }` → 追加一个闭包实参。
   `f(a) { }` 是给既有调用补上最后一个实参（f(a, { })）；否则新建单实参调用。 */
static bool is_callable_trailing(const AstNode *n) {
    return n->kind == AST_IDENT || n->kind == AST_MEMBER || n->kind == AST_CALL;
}

static AstNode *parse_trailing_closure(Parser *p, AstNode *callee) {
    const int line = callee->line, col = callee->col;
    AstNode *closure = parse_map_or_closure(p); /* cur 是 '{' */
    if (!closure) return NULL;
    if (callee->kind == AST_CALL) {
        AstArg *args = arena_alloc(p->arena, (callee->as.call.n_args + 1) * sizeof(AstArg));
        for (size_t i = 0; i < callee->as.call.n_args; i++) {
            args[i] = callee->as.call.args[i];
        }
        args[callee->as.call.n_args].named = false;
        args[callee->as.call.n_args].name = NULL;
        args[callee->as.call.n_args].expr = closure;
        callee->as.call.args = args;
        callee->as.call.n_args++;
        return callee;
    }
    AstArg *args = arena_alloc(p->arena, sizeof(AstArg));
    args[0].named = false;
    args[0].name = NULL;
    args[0].expr = closure;
    AstNode *n = ast_new(p->arena, AST_CALL, line, col);
    n->as.call.callee = callee;
    n->as.call.args = args;
    n->as.call.n_args = 1;
    return n;
}

/* 控制流条件/iterable/match 表达式处关闭尾随闭包，避免吞掉后续块。 */
static AstNode *parse_expr_no_trailing(Parser *p) {
    const bool saved = p->allow_trailing_closure;
    p->allow_trailing_closure = false;
    AstNode *e = parse_expr(p, 0);
    p->allow_trailing_closure = saved;
    return e;
}

/* 子表达式容器（括号/实参/数组元素/map 值）内始终允许尾随闭包，
   使 `if (arr.map { $0 > 0 }) { }` 这类嵌套写法不受外层 no_trailing 影响。 */
static AstNode *parse_expr_sub(Parser *p) {
    const bool saved = p->allow_trailing_closure;
    p->allow_trailing_closure = true;
    AstNode *e = parse_expr(p, 0);
    p->allow_trailing_closure = saved;
    return e;
}

static AstNode *parse_func_literal(Parser *p) {
    return parse_func(p, false, false);
}

/* ---------- 表达式主循环（Pratt） ---------- */

/* `is` 的右操作数：类型引用。支持内置类型名（error 是关键字需特判）、类名、
   枚举 case（Color.Red）。与普通表达式不同：关键字 error 也允许。 */
static AstNode *parse_type_ref(Parser *p) {
    const int line = p->cur.line, col = p->cur.col;
    if (p->cur.kind == TOK_ERR) { /* `is error`（error 是关键字） */
        advance(p);
        AstNode *n = ast_new(p->arena, AST_IDENT, line, col);
        n->as.ident.name = arena_copy_str(p->arena, "error", 5);
        return n;
    }
    if (p->cur.kind == TOK_NULL) { /* `is null`（null 是关键字） */
        advance(p);
        AstNode *n = ast_new(p->arena, AST_IDENT, line, col);
        n->as.ident.name = arena_copy_str(p->arena, "null", 4);
        return n;
    }
    if (p->cur.kind == TOK_ENUM) { /* `is enum`（enum 是关键字） */
        advance(p);
        AstNode *n = ast_new(p->arena, AST_IDENT, line, col);
        n->as.ident.name = arena_copy_str(p->arena, "enum", 4);
        return n;
    }
    if (p->cur.kind == TOK_COROUTINE) { /* `is coroutine`（coroutine 是关键字） */
        advance(p);
        AstNode *n = ast_new(p->arena, AST_IDENT, line, col);
        n->as.ident.name = arena_copy_str(p->arena, "coroutine", 9);
        return n;
    }
    if (p->cur.kind == TOK_IDENT) {
        AstNode *n = ast_new(p->arena, AST_IDENT, line, col);
        n->as.ident.name = arena_copy_str(p->arena, p->cur.start, p->cur.len);
        advance(p);
        if (check(p, TOK_DOT)) { /* 枚举 case：Color.Red */
            advance(p);
            const char *member = expect_ident(p, "expected case name after '.'");
            if (!member) return NULL;
            AstNode *m = ast_new(p->arena, AST_MEMBER, line, col);
            m->as.member.obj = n;
            m->as.member.name = member;
            return m;
        }
        return n;
    }
    error_here(p, "expected type name after 'is'");
    return NULL;
}

static AstNode *parse_expr(Parser *p, int min_prec) {
    if (++p->depth > MAX_PARSE_DEPTH) {
        error_here(p, "expression too deeply nested");
        p->depth--;
        return NULL;
    }

    AstNode *left = parse_prefix(p);
    if (!left) {
        p->depth--;
        return NULL;
    }

    for (;;) {
        if (check(p, TOK_LPAREN)) {
            left = parse_call_tail(p, left);
            continue;
        }
        if (check(p, TOK_LBRACKET)) {
            left = parse_index_tail(p, left);
            continue;
        }
        if (check(p, TOK_DOT_QUESTION)) {
            left = parse_optional_chain(p, left);
            if (!left) break;
            continue;
        }
        if (check(p, TOK_DOT)) {
            if (check_peek(p, TOK_TRY) || check_peek(p, TOK_TRY_QUESTION)) {
                left = parse_try_call(p, left);
                if (!left) break;
                continue;
            }
            left = parse_member_tail(p, left);
            if (!left) break; /* 成员名缺失：left 已无效，停止后续处理 */
            continue;
        }
        if (p->allow_trailing_closure && check(p, TOK_LBRACE) &&
            p->cur.line == p->prev_line && is_callable_trailing(left)) {
            left = parse_trailing_closure(p, left);
            if (!left) break;
            continue;
        }
        break;
    }

    while (!check(p, TOK_EOF)) {
        if (!left) break;
        /* `x !is T` = `!(x is T)`：`!` 不是二元运算符，须在 lookup_binop 前特判 */
        if (p->cur.kind == TOK_BANG && p->next.kind == TOK_IS) {
            advance(p); /* ! */
            advance(p); /* is */
            skip_newlines(p);
            AstNode *right = parse_type_ref(p);
            if (!right) break;
            AstNode *isn = ast_new(p->arena, AST_BINARY, left->line, left->col);
            isn->as.binary.op = TOK_IS;
            isn->as.binary.left = left;
            isn->as.binary.right = right;
            AstNode *notn = ast_new(p->arena, AST_UNARY, left->line, left->col);
            notn->as.unary.op = TOK_BANG;
            notn->as.unary.operand = isn;
            left = notn;
            continue;
        }
        const BinOp *bin = lookup_binop(p->cur.kind);
        if (!bin || bin->prec < min_prec) break;

        if (p->cur.kind == TOK_CATCH) {
            advance(p);
            skip_newlines(p);
            const CatchHandler handler = parse_catch_handler(p);
            AstNode *n = ast_new(p->arena, AST_CATCH, left->line, left->col);
            n->as.catch_.expr = left;
            n->as.catch_.handler = handler;
            left = n;
            continue;
        }
        if (p->cur.kind == TOK_QUESTION) {
            advance(p);
            skip_newlines(p);
            AstNode *then_ = parse_expr(p, 6);
            skip_newlines(p);
            if (!expect(p, TOK_COLON, "expected ':' in ternary expression")) break;
            skip_newlines(p);
            AstNode *else_ = parse_expr(p, bin->prec);
            AstNode *n = ast_new(p->arena, AST_TERNARY, left->line, left->col);
            n->as.ternary.cond = left;
            n->as.ternary.then_ = then_;
            n->as.ternary.else_ = else_;
            left = n;
            continue;
        }

        const int op = p->cur.kind;
        advance(p);
        skip_newlines(p);
        AstNode *right;
        if (op == TOK_IS) {
            right = parse_type_ref(p);
        } else {
            const int next_min = bin->right_assoc ? bin->prec : bin->prec + 1;
            right = parse_expr(p, next_min);
        }
        if (!right) break;
        AstNode *n = ast_new(p->arena, AST_BINARY, left->line, left->col);
        n->as.binary.op = op;
        n->as.binary.left = left;
        n->as.binary.right = right;
        left = n;
    }

    p->depth--;
    return left;
}

/* ---------- catch ---------- */

/* ---------- 模式匹配（catch 与 match 共用，解耦便于扩展） ---------- */

/* 解析 `(ident, ...)` 载荷名列表；无括号时 n_payload=0。 */
static const char **parse_payload_list(Parser *p, size_t *out_n) {
    const char **payload = NULL;
    size_t np = 0, cap = 0;
    if (match(p, TOK_LPAREN)) {
        p->paren_depth++;
        skip_newlines(p);
        while (!check(p, TOK_RPAREN) && !check(p, TOK_EOF)) {
            const char *pn = expect_ident(p, "expected payload identifier");
            if (!pn) break;
            ARR_PUSH(p, payload, np, cap, pn);
            skip_newlines(p);
            if (match(p, TOK_COMMA)) {
                skip_newlines(p);
                continue;
            }
            break;
        }
        skip_newlines(p);
        expect(p, TOK_RPAREN, "expected ')' after payload names");
        p->paren_depth--;
    }
    *out_n = np;
    return payload;
}

/* 通用模式解析。allow_bind=true（match）时裸标识符是绑定（ARM_BIND）；
   allow_bind=false（catch）时裸标识符是裸 case（ARM_ENUM）。 */
static AstMatchArm parse_pattern(Parser *p, bool allow_bind) {
    AstMatchArm arm = {0};
    switch (p->cur.kind) {
        case TOK_NUMBER: {
            arm.pattern = ARM_LITERAL;
            AstNode *n = ast_new(p->arena, AST_NUMBER, p->cur.line, p->cur.col);
            n->as.number.text = arena_copy_str(p->arena, p->cur.start, p->cur.len);
            n->as.number.len = p->cur.len;
            arm.literal = n;
            advance(p);
            return arm;
        }
        case TOK_STRING:
            arm.pattern = ARM_LITERAL;
            arm.literal = parse_string(p);
            return arm;
        case TOK_TRUE:
        case TOK_FALSE:
        case TOK_NULL: {
            arm.pattern = ARM_LITERAL;
            const AstKind k = (p->cur.kind == TOK_TRUE)   ? AST_TRUE
                              : (p->cur.kind == TOK_FALSE) ? AST_FALSE : AST_NULL;
            arm.literal = ast_new(p->arena, k, p->cur.line, p->cur.col);
            advance(p);
            return arm;
        }
        case TOK_IDENT: {
            const char *first = arena_copy_str(p->arena, p->cur.start, p->cur.len);
            advance(p);
            skip_newlines(p);
            if (strcmp(first, "_") == 0) {
                arm.pattern = ARM_WILDCARD;
                return arm;
            }
            if (check(p, TOK_DOT)) {
                advance(p);
                arm.pattern = ARM_ENUM;
                arm.set_name = first;
                arm.case_name = expect_ident(p, "expected case name after '.'");
                skip_newlines(p);
            } else if (check(p, TOK_LPAREN)) {
                arm.pattern = ARM_ENUM;
                arm.case_name = first;
            } else if (!allow_bind) {
                arm.pattern = ARM_ENUM;
                arm.case_name = first;
                return arm;
            } else {
                arm.pattern = ARM_BIND;
                arm.bind = first;
                return arm;
            }
            arm.payload = parse_payload_list(p, &arm.n_payload);
            return arm;
        }
        default:
            error_here(p, "expected match pattern");
            return arm;
    }
}

static AstNode *parse_match_arm_value(Parser *p) {
    skip_newlines(p);
    if (check(p, TOK_LBRACE)) {
        return parse_block(p);
    }
    return parse_expr(p, 0);
}

static CatchHandler parse_catch_match(Parser *p) {
    /* cur 是 '{' */
    CatchHandler h = {0};
    h.kind = CATCH_MATCH;
    advance(p);
    p->paren_depth++;
    skip_newlines(p);
    AstMatchArm *arms = NULL;
    size_t n = 0, cap = 0;
    while (!check(p, TOK_RBRACE) && !check(p, TOK_EOF)) {
        AstMatchArm arm = parse_pattern(p, false);
        if (arm.pattern != ARM_ENUM && arm.pattern != ARM_WILDCARD) {
            error_here(p, "catch patterns must be error cases");
        }
        skip_newlines(p);
        if (!expect(p, TOK_ARROW, "expected '=>' in match arm")) break;
        arm.value = parse_match_arm_value(p);
        ARR_PUSH(p, arms, n, cap, arm);
        skip_newlines(p);
        if (match(p, TOK_COMMA)) {
            skip_newlines(p);
            continue;
        }
        break;
    }
    skip_newlines(p);
    expect(p, TOK_RBRACE, "expected '}' after match arms");
    p->paren_depth--;
    h.arms = arms;
    h.n_arms = n;
    return h;
}

/* `match expr { 模式 => 值, ... }`：表达式或语句（由调用位置决定）。 */
static AstNode *parse_match(Parser *p) {
    const int line = p->cur.line, col = p->cur.col;
    advance(p); /* match */
    skip_newlines(p);
    AstNode *expr = parse_expr_no_trailing(p);
    if (!expr) return NULL;
    skip_newlines(p);
    if (!expect(p, TOK_LBRACE, "expected '{' after match expression")) return NULL;
    p->paren_depth++;
    skip_newlines(p);
    AstMatchArm *arms = NULL;
    size_t n = 0, cap = 0;
    while (!check(p, TOK_RBRACE) && !check(p, TOK_EOF)) {
        AstMatchArm arm = parse_pattern(p, true);
        skip_newlines(p);
        if (!expect(p, TOK_ARROW, "expected '=>' in match arm")) break;
        arm.value = parse_match_arm_value(p);
        ARR_PUSH(p, arms, n, cap, arm);
        skip_newlines(p);
        if (match(p, TOK_COMMA)) {
            skip_newlines(p);
            continue;
        }
        break;
    }
    skip_newlines(p);
    expect(p, TOK_RBRACE, "expected '}' after match arms");
    p->paren_depth--;
    AstNode *node = ast_new(p->arena, AST_MATCH, line, col);
    node->as.match_.expr = expr;
    node->as.match_.arms = arms;
    node->as.match_.n_arms = n;
    return node;
}

/* `throw expr`：抛出 error 值（语句）。 */
static AstNode *parse_throw(Parser *p) {
    const int line = p->cur.line, col = p->cur.col;
    advance(p); /* throw */
    skip_newlines(p);
    AstNode *expr = parse_expr(p, 0);
    if (!expr) return NULL;
    AstNode *n = ast_new(p->arena, AST_THROW, line, col);
    n->as.throw_.expr = expr;
    return n;
}

static CatchHandler parse_catch_handler(Parser *p) {
    if (check(p, TOK_LBRACE)) {
        return parse_catch_match(p);
    }
    if (check(p, TOK_IDENT) && check_peek(p, TOK_LBRACE)) {
        CatchHandler h = {0};
        h.kind = CATCH_BLOCK;
        h.error_name = arena_copy_str(p->arena, p->cur.start, p->cur.len);
        advance(p);
        skip_newlines(p);
        h.fallback = parse_block(p);
        return h;
    }
    CatchHandler h = {0};
    h.kind = CATCH_FALLBACK;
    h.fallback = parse_expr(p, 6);
    return h;
}

/* ---------- 语句 ---------- */

static AstNode *parse_var_decl(Parser *p) {
    const bool is_const = (p->cur.kind == TOK_LET);
    const int line = p->cur.line, col = p->cur.col;
    advance(p);

    const char **names = NULL;
    size_t n_names = 0, cap1 = 0;
    AstNode **values = NULL;
    size_t n_values = 0, cap2 = 0;

    skip_newlines(p);
    const char *first = expect_ident(p, "expected variable name");
    if (!first) return NULL;
    ARR_PUSH(p, names, n_names, cap1, first);

    /* `let name => body`：只读计算属性（2026-08-11 起）。仅 let 允许，读取时求值。 */
    if (check(p, TOK_ARROW)) {
        if (!is_const) {
            error_at(p, line, col, "computed property requires 'let' (not 'var')");
        }
        advance(p); /* => */
        AstNode *body = parse_prop_body(p, line, col);
        AstNode *n = ast_new(p->arena, AST_PROP_DECL, line, col);
        n->as.prop_decl.name = first;
        n->as.prop_decl.body = body;
        n->as.prop_decl.is_static = false;
        return n;
    }

    AstNode *first_init = NULL;
    skip_newlines(p);
    if (match(p, TOK_ASSIGN)) {
        first_init = parse_expr(p, 0);
    }

    if (first_init != NULL) {
        /* 形式 A：每个名字带自己的初始化 */
        ARR_PUSH(p, values, n_values, cap2, first_init);
        for (;;) {
            if (!match(p, TOK_COMMA)) break; /* 先检查逗号，勿吃换行 */
            skip_newlines(p);
            const char *name = expect_ident(p, "expected variable name");
            if (!name) break;
            ARR_PUSH(p, names, n_names, cap1, name);
            AstNode *init = NULL;
            if (match(p, TOK_ASSIGN)) {
                skip_newlines(p);
                init = parse_expr(p, 0);
            }
            if (init == NULL) {
                error_at(p, line, col, "variable must be initialized");
                init = ast_new(p->arena, AST_UNINITIALIZED, line, col);
            }
            ARR_PUSH(p, values, n_values, cap2, init);
        }
    } else {
        /* 形式 B：名字列表，然后 = 值列表（多返回值/并行初始化） */
        for (;;) {
            if (!match(p, TOK_COMMA)) break; /* 先检查逗号，勿吃换行 */
            skip_newlines(p);
            const char *name = expect_ident(p, "expected variable name");
            if (!name) break;
            ARR_PUSH(p, names, n_names, cap1, name);
        }
        skip_newlines(p);
        if (!expect(p, TOK_ASSIGN, "expected '=' after variable names")) return NULL;
        skip_newlines(p);
        for (;;) {
            AstNode *v = parse_expr(p, 0);
            if (!v) break;
            ARR_PUSH(p, values, n_values, cap2, v);
            if (!match(p, TOK_COMMA)) break; /* 先检查逗号，勿吃换行 */
            skip_newlines(p);
        }
    }

    if (n_values == 0) {
        error_at(p, line, col, "variable must be initialized (use uninitialized explicitly)");
    }

    AstNode *n = ast_new(p->arena, AST_VAR_DECL, line, col);
    n->as.var_decl.names = names;
    n->as.var_decl.n_names = n_names;
    n->as.var_decl.values = values;
    n->as.var_decl.n_values = n_values;
    n->as.var_decl.is_const = is_const;
    return n;
}

static AstNode *parse_if(Parser *p) {
    const int line = p->cur.line, col = p->cur.col;
    advance(p); /* if */
    AstNode *cond = parse_expr_no_trailing(p);
    skip_newlines(p);
    AstNode *then_ = parse_block(p);
    AstNode **elif_conds = NULL, **elif_bodies = NULL;
    size_t n_elif = 0, cap_c = 0, cap_b = 0;
    AstNode *else_ = NULL;
    skip_newlines(p);
    while (check(p, TOK_ELIF)) {
        advance(p);
        AstNode *c = parse_expr_no_trailing(p);
        skip_newlines(p);
        AstNode *b = parse_block(p);
        if (n_elif == cap_c) {
            size_t ncap = cap_c ? cap_c * 2 : 4;
            AstNode **na = arena_alloc(p->arena, ncap * sizeof(AstNode *));
            if (elif_conds) memcpy(na, elif_conds, n_elif * sizeof(AstNode *));
            elif_conds = na;
            cap_c = ncap;
        }
        if (n_elif == cap_b) {
            size_t ncap = cap_b ? cap_b * 2 : 4;
            AstNode **na = arena_alloc(p->arena, ncap * sizeof(AstNode *));
            if (elif_bodies) memcpy(na, elif_bodies, n_elif * sizeof(AstNode *));
            elif_bodies = na;
            cap_b = ncap;
        }
        elif_conds[n_elif] = c;
        elif_bodies[n_elif] = b;
        n_elif++;
        skip_newlines(p);
    }
    if (match(p, TOK_ELSE)) {
        skip_newlines(p);
        else_ = parse_block(p);
    }
    AstNode *n = ast_new(p->arena, AST_IF, line, col);
    n->as.if_.cond = cond;
    n->as.if_.then_ = then_;
    n->as.if_.elif_conds = elif_conds;
    n->as.if_.elif_bodies = elif_bodies;
    n->as.if_.n_elif = n_elif;
    n->as.if_.else_ = else_;
    return n;
}

static AstNode *parse_while(Parser *p) {
    const int line = p->cur.line, col = p->cur.col;
    advance(p); /* while */
    AstNode *cond = parse_expr_no_trailing(p);
    skip_newlines(p);
    AstNode *body = parse_block(p);
    AstNode *n = ast_new(p->arena, AST_WHILE, line, col);
    n->as.while_.cond = cond;
    n->as.while_.body = body;
    return n;
}

static AstNode *parse_for(Parser *p) {
    const int line = p->cur.line, col = p->cur.col;
    advance(p); /* for */
    const char **vars = NULL;
    size_t n_vars = 0, cap = 0;
    skip_newlines(p);
    for (;;) {
        const char *v = expect_ident(p, "expected loop variable");
        if (!v) break;
        ARR_PUSH(p, vars, n_vars, cap, v);
        skip_newlines(p);
        if (n_vars >= 2 && check(p, TOK_COMMA)) {
            error_here(p, "too many loop variables");
            break;
        }
        if (match(p, TOK_COMMA)) {
            skip_newlines(p);
            continue;
        }
        break;
    }
    if (!expect(p, TOK_IN, "expected 'in' in for loop")) return NULL;
    AstNode *iterable = parse_expr_no_trailing(p);
    skip_newlines(p);
    AstNode *step = NULL;
    if (match(p, TOK_STEP)) {
        skip_newlines(p);
        step = parse_expr_no_trailing(p);
        skip_newlines(p);
    }
    AstNode *body = parse_block(p);
    AstNode *n = ast_new(p->arena, AST_FOR_IN, line, col);
    n->as.for_in.vars = vars;
    n->as.for_in.n_vars = n_vars;
    n->as.for_in.iterable = iterable;
    n->as.for_in.step = step;
    n->as.for_in.body = body;
    return n;
}

static AstNode *parse_return(Parser *p) {
    const int line = p->cur.line, col = p->cur.col;
    advance(p); /* return */
    /* 注意：不能跳过换行——换行即裸 return（值须在同一行）。 */
    AstNode **values = NULL;
    size_t n = 0, cap = 0;
    if (!check(p, TOK_NEWLINE) && !check(p, TOK_EOF) && !check(p, TOK_RBRACE) &&
        !check(p, TOK_SEMICOLON)) {
        for (;;) {
            AstNode *v = parse_expr(p, 0);
            if (!v) break;
            ARR_PUSH(p, values, n, cap, v);
            if (!match(p, TOK_COMMA)) break; /* 先检查逗号，勿吃换行 */
            skip_newlines(p);
        }
    }
    AstNode *node = ast_new(p->arena, AST_RETURN, line, col);
    node->as.return_.values = values;
    node->as.return_.n = n;
    return node;
}

static AstNode *parse_expr_or_assign(Parser *p) {
    const int line = p->cur.line, col = p->cur.col;
    AstNode *first = parse_expr(p, 0);
    if (!first) return NULL;

    if (is_assign_op(p->cur.kind)) {
        const int op = p->cur.kind;
        advance(p);
        skip_newlines(p);
        AstNode *v = parse_expr(p, 0);
        AstNode **targets = arena_alloc(p->arena, sizeof(AstNode *));
        AstNode **values = arena_alloc(p->arena, sizeof(AstNode *));
        targets[0] = first;
        values[0] = v;
        AstNode *node = ast_new(p->arena, AST_ASSIGN, line, col);
        node->as.assign.targets = targets;
        node->as.assign.values = values;
        node->as.assign.n = 1;
        node->as.assign.n_values = 1;
        node->as.assign.op = op;
        return node;
    }

    if (check(p, TOK_COMMA)) {
        AstNode **targets = NULL;
        size_t n_targets = 0, cap_t = 0;
        ARR_PUSH(p, targets, n_targets, cap_t, first);
        for (;;) {
            if (!match(p, TOK_COMMA)) break; /* 先检查逗号，勿吃换行 */
            skip_newlines(p);
            AstNode *t = parse_expr(p, 0);
            if (!t) break;
            ARR_PUSH(p, targets, n_targets, cap_t, t);
        }
        if (is_assign_op(p->cur.kind)) {
            const int op = p->cur.kind;
            advance(p);
            skip_newlines(p);
            AstNode **values = NULL;
            size_t n_values = 0, cap_v = 0;
            for (;;) {
                AstNode *v = parse_expr(p, 0);
                if (!v) break;
                ARR_PUSH(p, values, n_values, cap_v, v);
                if (!match(p, TOK_COMMA)) break; /* 先检查逗号，勿吃换行 */
                skip_newlines(p);
            }
            AstNode *node = ast_new(p->arena, AST_ASSIGN, line, col);
            node->as.assign.targets = targets;
            node->as.assign.values = values;
            node->as.assign.n = n_targets;
            node->as.assign.n_values = n_values;
            node->as.assign.op = op;
            return node;
        }
        error_at(p, line, col, "expected '=' for multiple assignment");
        return NULL;
    }

    AstNode *node = ast_new(p->arena, AST_EXPR_STMT, line, col);
    node->as.expr_stmt.expr = first;
    return node;
}

/* 块末句 `;` 结尾的调用语句 → no_value（值块不返回其值；`f();` 是副作用调用）。 */
static void mark_no_value_stmt(Parser *p, AstNode *s) {
    if (s && s->kind == AST_EXPR_STMT && s->as.expr_stmt.expr &&
        s->as.expr_stmt.expr->kind == AST_CALL && check(p, TOK_SEMICOLON)) {
        s->as.expr_stmt.no_value = true;
    }
}

/* 解析 `{ stmts }` 的内容部分（调用方已消费 `{` 或 `${`，cur 在首条语句）。
   返回语句数组，*out_n 为数量；消费到匹配的 `}`。
   ⚠️ 语句层也必须计入递归深度：只给 parse_expr 设护栏时，`{{{{...}}}}` 与嵌套
   `${}` 插值可以无限递归下去，直接让编译器的 C 栈溢出（实测 8000 层 {} 崩溃、
   5000 层正常）。这里在块入口检查并在出口恢复。 */
static AstNode **parse_brace_body(Parser *p, size_t *out_n) {
    /* 语句层嵌套上限（独立计数器，避免与表达式深度叠加误报）。
       经验值：宿主编译器线程栈约 8MB，单层 `{` 约耗数百字节 C 栈，3000 层会栈溢出
       （实测 exit 0xC00000FD），1500 层安全。此处留足余量。 */
    const bool too_deep = (++p->block_depth > MAX_BLOCK_DEPTH);
    if (too_deep) {
        error_here(p, "too deeply nested blocks (nesting limit exceeded)");
        p->fatal = true;
        p->block_depth--;
        *out_n = 0;
        return NULL;
    }
    AstNode **stmts = NULL;
    size_t n = 0, cap = 0;
    while (!check(p, TOK_RBRACE) && !check(p, TOK_EOF)) {
        const char *stmt_start = p->cur.start;
        AstNode *s = parse_stmt(p);
        mark_no_value_stmt(p, s);
        if (s) ARR_PUSH(p, stmts, n, cap, s);
        if (p->fatal) break;
        skip_semis(p);
        if (check(p, TOK_RBRACE) || check(p, TOK_EOF)) break;
        /* 仅当完全未推进（坏 token 未被消耗）时才同步，防死循环；
           成功语句后 cur 已推进，绝不能吞掉下一条语句。 */
        if (p->cur.start == stmt_start &&
            !is_stmt_start(p->cur.kind) && !check(p, TOK_NEWLINE) && !check(p, TOK_SEMICOLON)) {
            synchronize(p);
        }
    }
    skip_newlines(p);
    expect(p, TOK_RBRACE, "expected '}' after block");
    p->block_depth--;
    *out_n = n;
    return stmts;
}

static AstNode *parse_block(Parser *p) {
    const int line = p->cur.line, col = p->cur.col;
    if (!expect(p, TOK_LBRACE, "expected '{'")) return NULL;
    p->paren_depth++;
    skip_newlines(p);
    size_t n = 0;
    AstNode **stmts = parse_brace_body(p, &n);
    p->paren_depth--;
    AstNode *node = ast_new(p->arena, AST_BLOCK, line, col);
    node->as.block.stmts = stmts;
    node->as.block.n = n;
    return node;
}

/* `${ ... }` 块表达式（2026-08-11 起，原 `get { ... }` 语法）：末句表达式作值；
   `;` 结尾的调用不产值；块内禁用 $N 隐式参数。cur 在 `${` 处。 */
static AstNode *parse_block_expr(Parser *p) {
    const int line = p->cur.line, col = p->cur.col;
    advance(p); /* 消费 ${ */
    p->paren_depth++;
    skip_newlines(p);
    size_t n = 0;
    AstNode **stmts = parse_brace_body(p, &n);
    p->paren_depth--;
    AstNode *node = ast_new(p->arena, AST_BLOCK_EXPR, line, col);
    node->as.block.stmts = stmts;
    node->as.block.n = n;
    if (scan_implicit_arg_max(node) >= 0) {
        error_at(p, line, col,
                 "implicit parameters ($0, $1, ...) are not allowed in a block expression");
    }
    return node;
}

static void parse_params(Parser *p, AstParam **out_params, size_t *out_n, bool *out_varargs) {
    AstParam *params = NULL;
    size_t n = 0, cap = 0;
    bool varargs = false;
    if (!expect(p, TOK_LPAREN, "expected '('")) return;
    p->paren_depth++;
    skip_newlines(p);
    while (!check(p, TOK_RPAREN) && !check(p, TOK_EOF)) {
        if (check(p, TOK_DOT_DOT_DOT)) {
            advance(p);
            varargs = true;
            break;
        }
        const char *name = expect_ident(p, "expected parameter name");
        if (!name) break;
        AstParam param = {0};
        param.name = name;
        skip_newlines(p);
        if (check(p, TOK_COLON)) {
            advance(p);
            skip_newlines(p);
            if (ident_is(p, "self")) {
                param.is_self = true;
                advance(p);
            } else {
                error_here(p, "expected 'self' after ':' in parameter");
            }
        } else if (match(p, TOK_ASSIGN)) {
            skip_newlines(p);
            param.has_default = true;
            param.default_expr = parse_expr(p, 0);
        }
        ARR_PUSH(p, params, n, cap, param);
        skip_newlines(p);
        if (match(p, TOK_COMMA)) {
            skip_newlines(p);
            continue;
        }
        break;
    }
    skip_newlines(p);
    expect(p, TOK_RPAREN, "expected ')' after parameters");
    p->paren_depth--;
    *out_params = params;
    *out_n = n;
    *out_varargs = varargs;
}

/* 运算符重载：读 operator 后的运算符 token（`[]` 特判为 LBRACKET）。 */
static TokenKind parse_operator_token(Parser *p) {
    const TokenKind k = p->cur.kind;
    switch (k) {
        case TOK_PLUS: case TOK_MINUS: case TOK_STAR: case TOK_SLASH:
        case TOK_PERCENT: case TOK_STAR_STAR: case TOK_SLASH_SLASH:
        case TOK_EQ: case TOK_BANG_EQ: case TOK_LT: case TOK_LE:
        case TOK_GT: case TOK_GE:
            advance(p);
            return k;
        case TOK_LBRACKET: /* operator[] */
            advance(p);
            if (!expect(p, TOK_RBRACKET, "expected ']' after 'operator['")) return TOK_EOF;
            return TOK_LBRACKET;
        default:
            error_here(p, "unsupported operator overload");
            return TOK_EOF;
    }
}

/* 运算符 + 参数个数（含 self）→ 内部魔术方法名。返回 NULL=参数个数非法。 */
static const char *operator_internal_name(TokenKind op, size_t n_params) {
    switch (op) {
        case TOK_PLUS: return "__add__";
        case TOK_MINUS:
            return n_params == 1 ? "__neg__" : (n_params == 2 ? "__sub__" : NULL);
        case TOK_STAR: return "__mul__";
        case TOK_SLASH: return "__div__";
        case TOK_SLASH_SLASH: return "__idiv__";
        case TOK_PERCENT: return "__mod__";
        case TOK_STAR_STAR: return "__pow__";
        case TOK_EQ: return "__eq__";
        case TOK_BANG_EQ: return "__ne__";
        case TOK_LT: return "__lt__";
        case TOK_LE: return "__le__";
        case TOK_GT: return "__gt__";
        case TOK_GE: return "__ge__";
        case TOK_LBRACKET: /* []：2参读、3参写 */
            return n_params == 2 ? "__index__" : (n_params == 3 ? "__index_set__" : NULL);
        default: return NULL;
    }
}

static AstNode *parse_func(Parser *p, bool require_name, bool is_static) {
    const int line = p->cur.line, col = p->cur.col;
    advance(p); /* func */
    const char *name = NULL;
    TokenKind op = TOK_EOF;
    if (require_name) {
        if (check(p, TOK_OPERATOR)) {
            advance(p); /* operator */
            op = parse_operator_token(p);
        } else {
            name = expect_ident(p, "expected function name");
        }
    } else if (check(p, TOK_IDENT)) {
        name = expect_ident(p, NULL);
    }
    skip_newlines(p);
    AstParam *params = NULL;
    size_t n_params = 0;
    bool has_varargs = false;
    parse_params(p, &params, &n_params, &has_varargs);
    if (op != TOK_EOF) {
        name = operator_internal_name(op, n_params);
        if (!name) {
            error_here(p, "invalid parameter count for operator overload");
            name = "__invalid__";
        }
    }
    bool is_throws = false;
    skip_newlines(p);
    if (match(p, TOK_THROWS)) {
        is_throws = true;
        skip_newlines(p);
    }
    AstNode *body = parse_block(p);
    AstNode *n = ast_new(p->arena, AST_FUNC_DECL, line, col);
    n->as.func.name = name;
    n->as.func.params = params;
    n->as.func.n_params = n_params;
    n->as.func.has_varargs = has_varargs;
    n->as.func.is_static = is_static;
    n->as.func.is_throws = is_throws;
    n->as.func.body = body;
    return n;
}

/* 当前 token 文本是否等于 s（用于类体内 get/set 上下文识别）。 */
static bool tok_text_is(const Parser *p, const char *s) {
    const size_t n = strlen(s);
    return p->cur.len == n && strncmp(p->cur.start, s, n) == 0;
}

/* 属性声明：`get name(self) {}` 或 `set name(self, value) {}`。
   借鉴 JS getter/setter；mica 风格方法须显式带 self 参数。 */
static AstNode *parse_property(Parser *p, bool is_setter) {
    const int line = p->cur.line, col = p->cur.col;
    advance(p); /* get / set */
    const char *name = expect_ident(p, "expected property name");
    skip_newlines(p);
    AstParam *params = NULL;
    size_t n_params = 0;
    bool has_varargs = false;
    parse_params(p, &params, &n_params, &has_varargs);
    bool is_throws = false;
    skip_newlines(p);
    if (match(p, TOK_THROWS)) {
        is_throws = true;
        skip_newlines(p);
    }
    AstNode *body = parse_block(p);
    AstNode *n = ast_new(p->arena, AST_FUNC_DECL, line, col);
    n->as.func.name = name;
    n->as.func.params = params;
    n->as.func.n_params = n_params;
    n->as.func.has_varargs = has_varargs;
    n->as.func.is_static = false;
    n->as.func.is_throws = is_throws;
    n->as.func.is_property = true;
    n->as.func.is_setter = is_setter;
    n->as.func.body = body;
    return n;
}

/* `=> body`：计算属性体（表达式或块）。体是函数体语义：单表达式隐式返回，多语句须 return。
   块内禁用 $N 隐式参数。 */
static AstNode *parse_prop_body(Parser *p, int line, int col) {
    skip_newlines(p);
    AstNode *body;
    if (check(p, TOK_LBRACE)) {
        body = parse_block(p);
    } else {
        body = parse_expr(p, 0);
    }
    if (body && scan_implicit_arg_max(body) >= 0) {
        error_at(p, line, col,
                 "implicit parameters ($0, $1, ...) are not allowed in a computed property");
    }
    if (body && parser_ast_has_yield(body)) {
        error_at(p, line, col, "computed property cannot contain 'yield'");
    }
    return body;
}

/* `let r => body` 在类内 → getter 方法（隐式 self；static 无 self）。 */
static AstNode *make_getter_prop(Parser *p, int line, int col, const char *name, AstNode *body,
                                 bool is_static) {
    AstParam *params = NULL;
    size_t n_params = 0;
    if (!is_static) {
        params = arena_alloc(p->arena, sizeof(AstParam));
        params[0].name = arena_copy_str(p->arena, "self", 4);
        params[0].is_self = true;
        params[0].has_default = false;
        params[0].default_expr = NULL;
        n_params = 1;
    }
    AstNode *n = ast_new(p->arena, AST_FUNC_DECL, line, col);
    n->as.func.name = name;
    n->as.func.params = params;
    n->as.func.n_params = n_params;
    n->as.func.has_varargs = false;
    n->as.func.is_static = is_static;
    n->as.func.is_throws = false;
    n->as.func.is_property = true;
    n->as.func.is_setter = false;
    n->as.func.body = body;
    return n;
}

static AstNode *parse_field(Parser *p) {
    const bool is_const = (p->cur.kind == TOK_LET);
    const int line = p->cur.line, col = p->cur.col;
    advance(p); /* var / let */
    const char *name = expect_ident(p, "expected field name");
    skip_newlines(p);
    /* `let name => body`（类内计算属性）→ getter */
    if (is_const && check(p, TOK_ARROW)) {
        advance(p); /* => */
        AstNode *body = parse_prop_body(p, line, col);
        return make_getter_prop(p, line, col, name, body, false);
    }
    AstNode *init = NULL;
    if (match(p, TOK_ASSIGN)) {
        init = parse_expr(p, 0);
    }
    AstNode *n = ast_new(p->arena, AST_VAR_DECL, line, col);
    const char **names = NULL;
    size_t nn = 0, cap_n = 0;
    ARR_PUSH(p, names, nn, cap_n, name);
    n->as.var_decl.names = names;
    n->as.var_decl.n_names = 1;
    AstNode **values = NULL;
    if (init) {
        size_t nv = 0, cap_v = 0;
        ARR_PUSH(p, values, nv, cap_v, init);
        n->as.var_decl.values = values;
        n->as.var_decl.n_values = 1;
    } else {
        n->as.var_decl.values = NULL;
        n->as.var_decl.n_values = 0;
    }
    n->as.var_decl.is_const = is_const;
    return n;
}

static AstNode *parse_class(Parser *p) {
    const int line = p->cur.line, col = p->cur.col;
    bool is_extern = match(p, TOK_EXTERN); /* extern class：编译器开洞，实现由运行时提供 */
    advance(p); /* class */
    const char *name = expect_ident(p, "expected class name");
    skip_newlines(p);
    const char *parent = NULL;
    if (match(p, TOK_COLON)) {
        skip_newlines(p);
        parent = expect_ident(p, "expected parent class name");
        skip_newlines(p);
    }
    if (!expect(p, TOK_LBRACE, "expected '{' after class name")) return NULL;
    p->paren_depth++;
    skip_newlines(p);
    AstNode **members = NULL;
    size_t n = 0, cap = 0;
    while (!check(p, TOK_RBRACE) && !check(p, TOK_EOF)) {
        AstNode *m = NULL;
        bool member_extern = false;
        if (check(p, TOK_EXTERN)) {
            if (!is_extern) {
                error_here(p, "'extern' member is only allowed in 'extern class'");
                synchronize(p);
                continue;
            }
            member_extern = true;
            advance(p);
            skip_newlines(p);
        }
        if (check(p, TOK_VAR) || check(p, TOK_LET)) {
            m = parse_field(p);
        } else if (check(p, TOK_STATIC)) {
            advance(p);
            skip_newlines(p);
            if (check(p, TOK_FUNC)) {
                m = parse_func(p, true, true);
            } else if (check(p, TOK_LET)) {
                /* static let r => body：静态计算属性 getter（无 self） */
                const int pline = p->cur.line, pcol = p->cur.col;
                advance(p); /* let */
                const char *pname = expect_ident(p, "expected property name");
                AstNode *pbody = NULL;
                if (pname && match(p, TOK_ARROW)) {
                    pbody = parse_prop_body(p, pline, pcol);
                } else if (pname) {
                    error_here(p, "expected '=>' after computed property name");
                }
                m = pname ? make_getter_prop(p, pline, pcol, pname, pbody, true) : NULL;
            } else {
                error_here(p, "expected 'func' or 'let' after 'static'");
                synchronize(p);
            }
        } else if (check(p, TOK_FUNC)) {
            m = parse_func(p, true, false);
        } else if (check(p, TOK_IDENT) &&
                   (tok_text_is(p, "get") || tok_text_is(p, "set"))) {
            m = parse_property(p, tok_text_is(p, "set"));
        } else {
            if (member_extern) error_here(p, "expected 'var'/'func'/'static' after 'extern'");
            else error_here(p, "unexpected token in class body");
            synchronize(p);
        }
        if (m) ARR_PUSH(p, members, n, cap, m);
        if (p->fatal) break;
        skip_semis(p);
    }
    skip_newlines(p);
    expect(p, TOK_RBRACE, "expected '}' after class body");
    p->paren_depth--;
    AstNode *node = ast_new(p->arena, AST_CLASS_DECL, line, col);
    node->as.class_.name = name;
    node->as.class_.parent = parent;
    node->as.class_.members = members;
    node->as.class_.n_members = n;
    node->as.class_.is_extern = is_extern;
    return node;
}

static AstNode *parse_error_set(Parser *p) {
    const int line = p->cur.line, col = p->cur.col;
    advance(p); /* error */
    const char *name = expect_ident(p, "expected error set name");
    skip_newlines(p);
    if (!expect(p, TOK_LBRACE, "expected '{' after error set name")) return NULL;
    p->paren_depth++;
    skip_newlines(p);
    AstErrorCase *cases = NULL;
    size_t n = 0, cap = 0;
    while (!check(p, TOK_RBRACE) && !check(p, TOK_EOF)) {
        AstErrorCase c = {0};
        c.name = expect_ident(p, "expected error case name");
        if (!c.name) break;
        skip_newlines(p);
        if (match(p, TOK_LPAREN)) {
            p->paren_depth++;
            skip_newlines(p);
            const char **payload = NULL;
            size_t np = 0, cap_p = 0;
            while (!check(p, TOK_RPAREN) && !check(p, TOK_EOF)) {
                const char *pn = expect_ident(p, "expected payload identifier");
                if (!pn) break;
                ARR_PUSH(p, payload, np, cap_p, pn);
                skip_newlines(p);
                if (match(p, TOK_COMMA)) {
                    skip_newlines(p);
                    continue;
                }
                break;
            }
            skip_newlines(p);
            expect(p, TOK_RPAREN, "expected ')' after payload");
            p->paren_depth--;
            c.payload = payload;
            c.n_payload = np;
        }
        ARR_PUSH(p, cases, n, cap, c);
        skip_newlines(p);
        if (match(p, TOK_COMMA)) {
            skip_newlines(p);
            continue;
        }
        break;
    }
    skip_newlines(p);
    expect(p, TOK_RBRACE, "expected '}' after error set");
    p->paren_depth--;
    AstNode *node = ast_new(p->arena, AST_ERROR_SET_DECL, line, col);
    node->as.error_set.name = name;
    node->as.error_set.cases = cases;
    node->as.error_set.n_cases = n;
    return node;
}

/* `enum Name { A, B = 5, C }`：成员可选显式值（自动递增到上一个成员 + 1）。 */
static AstNode *parse_enum(Parser *p) {
    const int line = p->cur.line, col = p->cur.col;
    advance(p); /* enum */
    const char *name = expect_ident(p, "expected enum name");
    skip_newlines(p);
    if (!expect(p, TOK_LBRACE, "expected '{' after enum name")) return NULL;
    p->paren_depth++;
    skip_newlines(p);
    AstEnumMember *members = NULL;
    size_t n = 0, cap = 0;
    while (!check(p, TOK_RBRACE) && !check(p, TOK_EOF)) {
        AstEnumMember m = {0};
        m.name = expect_ident(p, "expected enum member name");
        if (!m.name) break;
        skip_newlines(p);
        m.payload = parse_payload_list(p, &m.n_payload);
        skip_newlines(p);
        if (match(p, TOK_ASSIGN)) {
            skip_newlines(p);
            m.value = parse_expr(p, 0);
            if (!m.value) break;
        }
        ARR_PUSH(p, members, n, cap, m);
        skip_newlines(p);
        if (match(p, TOK_COMMA)) {
            skip_newlines(p);
            continue;
        }
        break;
    }
    skip_newlines(p);
    expect(p, TOK_RBRACE, "expected '}' after enum");
    p->paren_depth--;
    AstNode *node = ast_new(p->arena, AST_ENUM_DECL, line, col);
    node->as.enum_.name = name;
    node->as.enum_.members = members;
    node->as.enum_.n_members = n;
    return node;
}

static const char *parse_dotted_name(Parser *p) {
    const char *parts[32];
    size_t n = 0;
    const char *first = expect_ident(p, "expected module name");
    if (!first) return NULL;
    parts[n++] = first;
    while (check(p, TOK_DOT) && !check_peek(p, TOK_LBRACE)) { /* 停在 `.{`（选择性导入） */
        if (n >= 32) {
            error_here(p, "module name too long");
            break;
        }
        advance(p); /* . */
        const char *part = expect_ident(p, "expected identifier after '.'");
        if (!part) break;
        parts[n++] = part;
    }
    size_t total = 0;
    for (size_t i = 0; i < n; i++) total += strlen(parts[i]);
    total += (n > 0 ? n - 1 : 0);
    char *buf = arena_alloc(p->arena, total + 1);
    size_t off = 0;
    for (size_t i = 0; i < n; i++) {
        if (i) buf[off++] = '.';
        const size_t l = strlen(parts[i]);
        memcpy(buf + off, parts[i], l);
        off += l;
    }
    buf[off] = '\0';
    return buf;
}

static AstNode *parse_import(Parser *p) {
    const int line = p->cur.line, col = p->cur.col;
    advance(p); /* import */
    const char *name = parse_dotted_name(p);
    const char **names = NULL;
    size_t n_names = 0, cap = 0;
    if (check(p, TOK_DOT)) { /* `import Mod.{ a, b }` 选择性导入 */
        advance(p); /* . */
        if (!expect(p, TOK_LBRACE, "expected '{' after import '.'")) return NULL;
        p->paren_depth++;
        skip_newlines(p);
        while (!check(p, TOK_RBRACE) && !check(p, TOK_EOF)) {
            const char *mn = expect_ident(p, "expected member name");
            if (!mn) break;
            ARR_PUSH(p, names, n_names, cap, mn);
            skip_newlines(p);
            if (match(p, TOK_COMMA)) {
                skip_newlines(p);
                continue;
            }
            break;
        }
        skip_newlines(p);
        expect(p, TOK_RBRACE, "expected '}' after import members");
        p->paren_depth--;
    }
    AstNode *n = ast_new(p->arena, AST_IMPORT, line, col);
    n->as.import.name = name;
    n->as.import.names = names;
    n->as.import.n_names = n_names;
    return n;
}

static AstNode *parse_export(Parser *p) {
    const int line = p->cur.line, col = p->cur.col;
    advance(p); /* export */
    if (check(p, TOK_IDENT) && tok_text_is(p, "module")) { /* module 是弱关键字（可作标识符） */
        advance(p);
        const char *name = parse_dotted_name(p);
        AstNode *n = ast_new(p->arena, AST_EXPORT_MODULE, line, col);
        n->as.export_module.name = name;
        return n;
    }
    AstNode *decl = parse_stmt(p);
    if (!decl) return NULL;
    AstNode *n = ast_new(p->arena, AST_EXPORT, line, col);
    n->as.export_.decl = decl;
    return n;
}

static AstNode *parse_stmt(Parser *p) {
    switch (p->cur.kind) {
        case TOK_VAR:
        case TOK_LET:
            return parse_var_decl(p);
        case TOK_DOLLAR_LBRACE: {
            AstNode *e = parse_block_expr(p);
            if (!e) return NULL;
            AstNode *n = ast_new(p->arena, AST_EXPR_STMT, e->line, e->col);
            n->as.expr_stmt.expr = e;
            return n;
        }
        case TOK_IF:
            return parse_if(p);
        case TOK_WHILE:
            return parse_while(p);
        case TOK_FOR:
            return parse_for(p);
        case TOK_BREAK: {
            AstNode *n = ast_new(p->arena, AST_BREAK, p->cur.line, p->cur.col);
            advance(p);
            return n;
        }
        case TOK_CONTINUE: {
            AstNode *n = ast_new(p->arena, AST_CONTINUE, p->cur.line, p->cur.col);
            advance(p);
            return n;
        }
        case TOK_RETURN:
            return parse_return(p);
        case TOK_DEFER: {
            const int line = p->cur.line, col = p->cur.col;
            advance(p);
            AstNode *block = parse_block(p);
            if (!block) return NULL;
            AstNode *n = ast_new(p->arena, AST_DEFER, line, col);
            n->as.defer_.block = block;
            return n;
        }
        case TOK_DO: {
            const int line = p->cur.line, col = p->cur.col;
            advance(p);
            AstNode *body = parse_block(p);
            if (!body) return NULL;
            skip_newlines(p);
            if (!check(p, TOK_CATCH)) {
                error_at(p, line, col, "do block requires a catch handler");
                return NULL;
            }
            advance(p); /* catch */
            skip_newlines(p);
            CatchHandler handler = parse_catch_handler(p);
            AstNode *n = ast_new(p->arena, AST_DO_CATCH, line, col);
            n->as.do_catch.body = body;
            n->as.do_catch.handler = handler;
            return n;
        }
        case TOK_TRY:
        case TOK_TRY_QUESTION:
        case TOK_TRY_BANG: {
            AstNode *e = parse_expr(p, 0);
            if (!e) return NULL;
            if (e->kind == AST_TRY_BLOCK) return e; /* try { } 作语句（不包 AST_EXPR_STMT） */
            AstNode *n = ast_new(p->arena, AST_EXPR_STMT, e->line, e->col);
            n->as.expr_stmt.expr = e;
            return n;
        }
        case TOK_LBRACE:
            return parse_block(p);
        case TOK_FUNC:
            if (check_peek(p, TOK_IDENT)) {
                return parse_func(p, true, false);
            }
            return parse_expr_or_assign(p);
        case TOK_CLASS:
            return parse_class(p);
        case TOK_EXTERN:
            if (check_peek(p, TOK_CLASS)) return parse_class(p);
            error_here(p, "expected 'class' after 'extern'");
            synchronize(p);
            return NULL;
        case TOK_ENUM:
            return parse_enum(p);
        case TOK_THROW:
            return parse_throw(p);
        case TOK_MATCH:
            return parse_match(p);
        case TOK_ERR:
            if (check_peek(p, TOK_IDENT)) {
                return parse_error_set(p);
            }
            return parse_expr_or_assign(p);
        case TOK_IMPORT:
            return parse_import(p);
        case TOK_YIELD: {
            AstNode *e = parse_expr(p, 0);
            if (!e) return NULL;
            AstNode *n = ast_new(p->arena, AST_EXPR_STMT, e->line, e->col);
            n->as.expr_stmt.expr = e;
            return n;
        }
        case TOK_EXPORT:
            return parse_export(p);
        case TOK_SEMICOLON:
            advance(p);
            return NULL;
        case TOK_NEWLINE:
            skip_newlines(p);
            return NULL;
        default:
            return parse_expr_or_assign(p);
    }
}

/* ---------- 对外 API ---------- */

static void init_tokens(Parser *p) {
    p->cur = lexer_next(p->lex);
    p->next = lexer_next(p->lex);
    p->prev_line = p->cur.line;
    /* 首个 token 若已是词法错误，立即上报（否则会被静默吞掉） */
    if (p->cur.kind == TOK_ERROR) {
        const LexError *e = lexer_error(p->lex);
        error_at(p, e->line, e->col, e->message);
        p->fatal = true;
    }
}

Parser *parser_new(Lexer *lex) {
    Parser *p = calloc(1, sizeof(Parser));
    assert(p != NULL);
    p->lex = lex;
    p->arena = arena_new();
    p->owns_arena = true;
    p->allow_trailing_closure = true;
    init_tokens(p);
    return p;
}

Parser *parser_new_shared(Lexer *lex, Arena *arena) {
    Parser *p = calloc(1, sizeof(Parser));
    assert(p != NULL);
    p->lex = lex;
    p->arena = arena;
    p->owns_arena = false;
    p->allow_trailing_closure = true;
    init_tokens(p);
    return p;
}

void parser_free(Parser *p) {
    if (!p) return;
    token_free(&p->cur);
    token_free(&p->next);
    if (p->owns_arena) arena_free(p->arena);
    free(p);
}

size_t parser_error_count(const Parser *p) {
    return p->n_errors;
}

const ParserError *parser_error(const Parser *p, size_t i) {
    return &p->errors[i];
}

AstNode **parser_parse(Parser *p, size_t *out_count) {
    AstNode **stmts = NULL;
    size_t n = 0, cap = 0;
    skip_newlines(p);
    while (!check(p, TOK_EOF) && !p->fatal) {
        if (check(p, TOK_ERROR)) break;
        const char *stmt_start = p->cur.start;
        AstNode *s = parse_stmt(p);
        if (s) ARR_PUSH(p, stmts, n, cap, s);
        if (p->fatal) break;
        skip_semis(p);
        if (check(p, TOK_EOF)) break;
        if (p->cur.start == stmt_start &&
            !is_stmt_start(p->cur.kind) && !check(p, TOK_NEWLINE) && !check(p, TOK_SEMICOLON)) {
            synchronize(p);
        }
    }
    *out_count = n;
    return stmts;
}
