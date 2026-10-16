#ifndef MICA_SEMA_H
#define MICA_SEMA_H

#include <stddef.h>

#include "ast.h"
#include "parser.h" /* ParserError */

/* 语义分析：注册标识符、解析作用域、校验命名参数、上下文检查。
   输入已解析成功的 AST（顶层语句数组），错误以 ParserError 形式收集。 */
typedef struct Sema Sema;

Sema *sema_new(void);
void sema_free(Sema *s);

/* 声明外部全局名（宿主注入或先前运行产生的全局），使 sema 不将其报为未定义。 */
void sema_add_external_global(Sema *s, const char *name);

void sema_analyze(Sema *s, AstNode **stmts, size_t n);

size_t sema_error_count(const Sema *s);
const ParserError *sema_error(const Sema *s, size_t i);

#endif /* MICA_SEMA_H */
