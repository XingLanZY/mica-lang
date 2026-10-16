#ifndef MICA_DIAG_H
#define MICA_DIAG_H

#include <stddef.h>

#include "lexer.h"

/* 以 GCC/Clang 风格打印词法错误：
   file:line:col: error: message
     行号 | 出错源码行（tab 展开、UTF-8 宽度感知、长行截断）
          | ^~~~~ 箭头覆盖出错 token
   source 为完整源码缓冲区（UTF-8），err 来自 lexer_error()。 */
void diag_print_error(const char *source, size_t len,
                      const char *filename, const LexError *err);

#endif /* MICA_DIAG_H */
