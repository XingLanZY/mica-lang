#include "token.h"

#include <stdlib.h>
#include <string.h>

#include "array.h"

const char *token_kind_name(TokenKind kind) {
    static const char *const names[] = {
        "EOF", "NEWLINE", "ERROR",
        "NUMBER", "STRING", "IDENT",
        "var", "let", "null", "true", "false", "uninitialized",
        "if", "elif", "else", "error", "while", "for", "in", "break", "continue",
        "func", "return", "class", "super", "import", "export", "extern",
        "try", "catch", "throws", "step", "static", "try?", "try!",
        "this_func", "yield", "do", "defer", "enum", "operator",
        "throw", "match", "is", "coroutine",
        "+", "-", "*", "/", "%", "**", "//",
        "+=", "-=", "*=", "/=", "%=", "**=", "//=",
        "=", "==", "!", "!=", "<", "<=", ">", ">=", "&&", "||",
        "?", "??", ":", "(", ")", "{", "}", "[", "]", "${",
        ",", ".", "?.", "..", "...", ";", "=>",
    };
    _Static_assert(sizeof(names) / sizeof(names[0]) == TOK_KIND_COUNT,
                   "token kind name table out of sync with TokenKind enum");
    if (kind < TOK_EOF || kind >= TOK_KIND_COUNT) return "<?>";
    return names[kind];
}

/* ---------- 关键字表 ---------- */

typedef struct {
    const char *text;
    size_t len;
    TokenKind kind;
} KeywordEntry;

static const KeywordEntry k_keywords[] = {
    {"break", sizeof("break") - 1, TOK_BREAK},
    {"catch", sizeof("catch") - 1, TOK_CATCH},
    {"class", sizeof("class") - 1, TOK_CLASS},
    {"continue", sizeof("continue") - 1, TOK_CONTINUE},
    {"coroutine", sizeof("coroutine") - 1, TOK_COROUTINE},
    {"defer", sizeof("defer") - 1, TOK_DEFER},
    {"do", sizeof("do") - 1, TOK_DO},
    {"elif", sizeof("elif") - 1, TOK_ELIF},
    {"else", sizeof("else") - 1, TOK_ELSE},
    {"enum", sizeof("enum") - 1, TOK_ENUM},
    {"error", sizeof("error") - 1, TOK_ERR},
    {"export", sizeof("export") - 1, TOK_EXPORT},
    {"extern", sizeof("extern") - 1, TOK_EXTERN},
    {"false", sizeof("false") - 1, TOK_FALSE},
    {"for", sizeof("for") - 1, TOK_FOR},
    {"func", sizeof("func") - 1, TOK_FUNC},
    {"if", sizeof("if") - 1, TOK_IF},
    {"import", sizeof("import") - 1, TOK_IMPORT},
    {"in", sizeof("in") - 1, TOK_IN},
    {"is", sizeof("is") - 1, TOK_IS},
    {"let", sizeof("let") - 1, TOK_LET},
    {"match", sizeof("match") - 1, TOK_MATCH},
    {"null", sizeof("null") - 1, TOK_NULL},
    {"operator", sizeof("operator") - 1, TOK_OPERATOR},
    {"return", sizeof("return") - 1, TOK_RETURN},
    {"static", sizeof("static") - 1, TOK_STATIC},
    {"step", sizeof("step") - 1, TOK_STEP},
    {"super", sizeof("super") - 1, TOK_SUPER},
    {"this_func", sizeof("this_func") - 1, TOK_THIS_FUNC},
    {"throw", sizeof("throw") - 1, TOK_THROW},
    {"throws", sizeof("throws") - 1, TOK_THROWS},
    {"true", sizeof("true") - 1, TOK_TRUE},
    {"try", sizeof("try") - 1, TOK_TRY},
    {"uninitialized", sizeof("uninitialized") - 1, TOK_UNINITIALIZED},
    {"var", sizeof("var") - 1, TOK_VAR},
    {"while", sizeof("while") - 1, TOK_WHILE},
    {"yield", sizeof("yield") - 1, TOK_YIELD},
};

TokenKind token_keyword_lookup(const char *s, size_t n) {
    for (size_t i = 0; i < sizeof(k_keywords) / sizeof(k_keywords[0]); i++) {
        const KeywordEntry *e = &k_keywords[i];
        if (e->len == n && memcmp(s, e->text, n) == 0) return e->kind;
    }
    return TOK_EOF;
}

/* ---------- 运算符表（最长匹配） ---------- */

typedef struct {
    const char *text;
    size_t len;
    TokenKind kind;
} OperatorEntry;

static const OperatorEntry k_operators[] = {
    {"...", sizeof("...") - 1, TOK_DOT_DOT_DOT},
    {"**=", sizeof("**=") - 1, TOK_STAR_STAR_ASSIGN},
    {"//=", sizeof("//=") - 1, TOK_SLASH_SLASH_ASSIGN},
    {"..", sizeof("..") - 1, TOK_DOT_DOT},
    {"?.", sizeof("?.") - 1, TOK_DOT_QUESTION},
    {"??", sizeof("??") - 1, TOK_QUESTION_QUESTION},
    {"**", sizeof("**") - 1, TOK_STAR_STAR},
    {"//", sizeof("//") - 1, TOK_SLASH_SLASH},
    {"+=", sizeof("+=") - 1, TOK_PLUS_ASSIGN},
    {"-=", sizeof("-=") - 1, TOK_MINUS_ASSIGN},
    {"*=", sizeof("*=") - 1, TOK_STAR_ASSIGN},
    {"/=", sizeof("/=") - 1, TOK_SLASH_ASSIGN},
    {"%=", sizeof("%=") - 1, TOK_PERCENT_ASSIGN},
    {"==", sizeof("==") - 1, TOK_EQ},
    {"!=", sizeof("!=") - 1, TOK_BANG_EQ},
    {"<=", sizeof("<=") - 1, TOK_LE},
    {">=", sizeof(">=") - 1, TOK_GE},
    {"&&", sizeof("&&") - 1, TOK_AND_AND},
    {"||", sizeof("||") - 1, TOK_OR_OR},
    {"${", sizeof("${") - 1, TOK_DOLLAR_LBRACE},
    {"=>", sizeof("=>") - 1, TOK_ARROW},
    {"+", sizeof("+") - 1, TOK_PLUS},
    {"-", sizeof("-") - 1, TOK_MINUS},
    {"*", sizeof("*") - 1, TOK_STAR},
    {"/", sizeof("/") - 1, TOK_SLASH},
    {"%", sizeof("%") - 1, TOK_PERCENT},
    {"=", sizeof("=") - 1, TOK_ASSIGN},
    {"!", sizeof("!") - 1, TOK_BANG},
    {"<", sizeof("<") - 1, TOK_LT},
    {">", sizeof(">") - 1, TOK_GT},
    {"?", sizeof("?") - 1, TOK_QUESTION},
    {":", sizeof(":") - 1, TOK_COLON},
    {"(", sizeof("(") - 1, TOK_LPAREN},
    {")", sizeof(")") - 1, TOK_RPAREN},
    {"{", sizeof("{") - 1, TOK_LBRACE},
    {"}", sizeof("}") - 1, TOK_RBRACE},
    {"[", sizeof("[") - 1, TOK_LBRACKET},
    {"]", sizeof("]") - 1, TOK_RBRACKET},
    {",", sizeof(",") - 1, TOK_COMMA},
    {".", sizeof(".") - 1, TOK_DOT},
    {";", sizeof(";") - 1, TOK_SEMICOLON},
};

TokenKind token_operator_match(const char *s, size_t max_len, size_t *matched_len) {
    const OperatorEntry *best = NULL;
    size_t best_len = 0;
    for (size_t i = 0; i < sizeof(k_operators) / sizeof(k_operators[0]); i++) {
        const OperatorEntry *e = &k_operators[i];
        if (e->len > max_len || e->len <= best_len) continue;
        if (memcmp(s, e->text, e->len) == 0) {
            best = e;
            best_len = e->len;
        }
    }
    if (!best) return TOK_EOF;
    *matched_len = best_len;
    return best->kind;
}

void string_value_free(StringValue *sv) {
    if (sv->segs == NULL) return;
    const size_t n = arr_size(sv->segs);
    for (size_t i = 0; i < n; i++) {
        StringSeg *seg = (StringSeg *)arr_at(sv->segs, i);
        if (seg->kind == SEG_LITERAL_STR) {
            str_free(&seg->data.str);
        }
    }
    arr_free(sv->segs);
    sv->segs = NULL;
}

void token_free(Token *tok) {
    if (tok->kind == TOK_STRING) {
        string_value_free(&tok->value.str);
    }
}
