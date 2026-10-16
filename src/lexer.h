#ifndef MICA_LEXER_H
#define MICA_LEXER_H

#include <stdbool.h>

#include "token.h"

typedef struct {
    bool has_error;
    int line;      /* 1 基 */
    int col;       /* 1 基，按字节 */
    int span;      /* 出错 token 的字节长度（>=1），用于下划线覆盖 */
    char message[256];
} LexError;

typedef struct Lexer Lexer;

/* 创建词法分析器。source 由调用方持有，分析期间不得释放。 */
Lexer *lexer_new(const char *source, size_t len);

/* 创建子串词法分析器，起始行列设为绝对位置（用于字符串插值：内部错误/AST
   行列能落到源码真实位置）。line/col 为 1 基、col 按字节。 */
Lexer *lexer_new_at(const char *source, size_t len, int line, int col);
void lexer_free(Lexer *lx);

/* 返回下一个 token。出错后首次返回 TOK_ERROR，之后恒返回 TOK_EOF。 */
Token lexer_next(Lexer *lx);

/* 词法错误信息（TOK_ERROR 后有效）。 */
const LexError *lexer_error(const Lexer *lx);

#endif /* MICA_LEXER_H */
