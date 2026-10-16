#ifndef MICA_TOKEN_H
#define MICA_TOKEN_H

#include <stddef.h>

#include "array.h"
#include "str.h"

typedef enum {
    /* 特殊 token */
    TOK_EOF,
    TOK_NEWLINE,
    TOK_ERROR,

    /* 字面量 */
    TOK_NUMBER,
    TOK_STRING,
    TOK_IDENT,

    /* 关键字 */
    TOK_VAR,
    TOK_LET,
    TOK_NULL,
    TOK_TRUE,
    TOK_FALSE,
    TOK_UNINITIALIZED,
    TOK_IF,
    TOK_ELIF,
    TOK_ELSE,
    TOK_ERR,               /* error：构造错误值 */
    TOK_WHILE,
    TOK_FOR,
    TOK_IN,
    TOK_BREAK,
    TOK_CONTINUE,
    TOK_FUNC,
    TOK_RETURN,
    TOK_CLASS,
    TOK_SUPER,
    TOK_IMPORT,
    TOK_EXPORT,
    TOK_EXTERN,          /* extern：extern class（编译器开洞的类，实现由运行时提供） */
    TOK_TRY,
    TOK_CATCH,
    TOK_THROWS,          /* throws：函数标注，表示可能返回错误 */
    TOK_STEP,
    TOK_STATIC,          /* static：无 self 的类方法 */
    TOK_TRY_QUESTION,    /* try?：失败得到 null */
    TOK_TRY_BANG,        /* try!：失败直接终止程序 */
    TOK_THIS_FUNC,       /* this_func：当前匿名函数自身（递归用） */
    TOK_YIELD,           /* yield：挂起 */
    TOK_DO,              /* do：do { } catch { } 错误处理块（语句，不可作表达式） */
    TOK_DEFER,           /* defer：延迟块 */
    TOK_ENUM,            /* enum：枚举声明（成员为自动递增常量，可带 payload） */
    TOK_OPERATOR,        /* operator：运算符重载（类方法 func operator+(self,o) {}） */
    TOK_THROW,           /* throw：抛出 error 值（所有 error 必须用 throw 返回，不用 return） */
    TOK_MATCH,           /* match：模式匹配表达式/语句（不能用于 error 类型） */
    TOK_IS,              /* is：类型判断（x is number / is Point / is Color.Red） */
    TOK_COROUTINE,       /* coroutine：关键字保留字（协程命名空间 coroutine.create 等） */

    /* 运算符与标点 */
    TOK_PLUS,              /* + */
    TOK_MINUS,             /* - */
    TOK_STAR,              /* * */
    TOK_SLASH,             /* / */
    TOK_PERCENT,           /* % */
    TOK_STAR_STAR,         /* ** */
    TOK_SLASH_SLASH,       /* // */
    TOK_PLUS_ASSIGN,       /* += */
    TOK_MINUS_ASSIGN,      /* -= */
    TOK_STAR_ASSIGN,       /* *= */
    TOK_SLASH_ASSIGN,      /* /= */
    TOK_PERCENT_ASSIGN,    /* %= */
    TOK_STAR_STAR_ASSIGN,  /* **= */
    TOK_SLASH_SLASH_ASSIGN,/* //= */
    TOK_ASSIGN,            /* = */
    TOK_EQ,                /* == */
    TOK_BANG,              /* ! */
    TOK_BANG_EQ,           /* != */
    TOK_LT,                /* < */
    TOK_LE,                /* <= */
    TOK_GT,                /* > */
    TOK_GE,                /* >= */
    TOK_AND_AND,           /* && */
    TOK_OR_OR,             /* || */
    TOK_QUESTION,          /* ? */
    TOK_QUESTION_QUESTION, /* ?? 空合并：a ?? b（a 非 null 取 a，否则 b） */
    TOK_COLON,             /* : */
    TOK_LPAREN,            /* ( */
    TOK_RPAREN,            /* ) */
    TOK_LBRACE,            /* { */
    TOK_RBRACE,            /* } */
    TOK_LBRACKET,          /* [ */
    TOK_RBRACKET,          /* ] */
    TOK_DOLLAR_LBRACE,     /* ${：块表达式（${ stmts }） */
    TOK_COMMA,             /* , */
    TOK_DOT,               /* . */
    TOK_DOT_QUESTION,      /* ?. 可选链：a?.b（a 为 null 得 null，否则 a.b） */
    TOK_DOT_DOT,           /* .. */
    TOK_DOT_DOT_DOT,       /* ... */
    TOK_SEMICOLON,         /* ; */
    TOK_ARROW,             /* => */

    TOK_KIND_COUNT
} TokenKind;

/* 字符串插值分段 */
typedef enum {
    SEG_LITERAL_VIEW,  /* 字面量段：无转义，为源码视图（不拷贝） */
    SEG_LITERAL_STR,   /* 字面量段：含转义，已解码为拥有的 String */
    SEG_INTERP         /* 插值表达式段：表达式原文视图（待 parser 解析） */
} StringSegKind;

typedef struct {
    StringSegKind kind;
    union {
        struct {
            const char *start;
            size_t len;
        } view;
        String str;   /* SEG_LITERAL_STR：解码后的字符串（需 str_free） */
    } data;
} StringSeg;

/* 字符串插值分段（由 Array 容器管理，元素为 StringSeg）。
   segs 为 NULL 表示无插值的普通字符串。 */
typedef struct {
    Array *segs;
} StringValue;

/* token 的 start/len 均指向源码缓冲区，不拷贝。 */
typedef struct {
    TokenKind kind;
    const char *start;   /* token 原文在源码中的起点 */
    size_t len;          /* 字节数；NEWLINE 为 0 */
    int line;            /* 1 基起始行 */
    int col;             /* 1 基起始列（按字节） */
    union {
        /* TOK_STRING：插值分段。count == 0 表示无插值的普通字符串，
           字面内容为 token 原文去掉引号（parser 负责去引号/解码）。 */
        StringValue str;
    } value;
} Token;

const char *token_kind_name(TokenKind kind);

/* 标识符命中关键字则返回对应 token kind，否则返回 TOK_EOF。 */
TokenKind token_keyword_lookup(const char *s, size_t n);

/* 运算符最长匹配：在 s 处匹配最长运算符，写入长度；无匹配返回 TOK_EOF。 */
TokenKind token_operator_match(const char *s, size_t max_len, size_t *matched_len);

/* 释放 StringValue 内部分段（含各 SEG_LITERAL_STR 拥有的 String 与 Array）。 */
void string_value_free(StringValue *sv);

/* 释放 token 内部堆内存。 */
void token_free(Token *tok);

#endif /* MICA_TOKEN_H */
