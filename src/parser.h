#ifndef MICA_PARSER_H
#define MICA_PARSER_H

#include <stddef.h>

#include "ast.h"
#include "lexer.h"

typedef struct {
    int line;
    int col;
    char message[256];
} ParserError;

typedef struct Parser Parser;

/* 创建/释放 parser。lex 由调用方持有（parser 不释放）。
   parser 拥有自己的 Arena；parser_free 时一并释放。 */
Parser *parser_new(Lexer *lex);
void parser_free(Parser *p);

/* 解析整个源码，返回顶层语句数组（arena 分配）。
   出错时仍返回已解析的部分；用 parser_error_* 查询错误。 */
AstNode **parser_parse(Parser *p, size_t *out_count);

size_t parser_error_count(const Parser *p);
const ParserError *parser_error(const Parser *p, size_t i);

/* 内部：共享 arena 的子 parser（用于字符串插值表达式）。 */
Parser *parser_new_shared(Lexer *lex, Arena *arena);

#endif /* MICA_PARSER_H */
