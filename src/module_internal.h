#ifndef MICA_MODULE_INTERNAL_H
#define MICA_MODULE_INTERNAL_H

/* 宿主层内部接口：mica.c（运行时 / 公开 API / 编译执行）与 module.c（模块与程序集加载）
   之间的共享声明。两者同属 L4 编排层，拆分的目的是让模块子系统独立成文件，
   而不是把内部实现互相暴露给整个项目。符号均为宿主层内部细节，不对宿主公开。
   生成方式：按源码实际签名摘录（改签名须同步本文件）。 */

#include "mica.h"
#include "ast.h"
#include "value.h"
#include "state.h"
#include "parser.h"
#include "lexer.h"

/* ---- 定义在 mica.c，供 module.c 复用 ---- */
char *dup_str(const char *s);
void str_set(char **dst, const char *fmt, ...);
void str_clear(char **dst);
bool file_exists(const char *path);
char *read_file(const char *path, size_t *out_len);
void list_add(const char ***arr, size_t *n, size_t *cap, const char *item);
void unmark_module(mica_runtime *rt, const char *name);
void add_search_path_front(mica_runtime *rt, const char *dir);
bool is_executable_stmt(const AstNode *n);

/* ---- 定义在 module.c，供 mica.c 复用 ---- */
bool get_global_value(mica_runtime *rt, const char *name, Value *out);
void set_global_value(mica_runtime *rt, const char *name, Value v);
void set_nested_namespace(mica_runtime *rt, const char *name, ObjMap *ns);
void discard_frames(mica_context *ctx);
int parse_source(const char *src, size_t len, Parser **out_p, Lexer **out_lx, AstNode ***out_stmts, size_t *out_n, char *err, size_t err_cap, int *out_line, int *out_col);
int load_module(mica_runtime *rt, const char *name);
int validate_entry(const AstNode *const *stmts, size_t n, char *err, size_t err_cap);
void collect_import_names(const AstNode *const *stmts, size_t n, const char ***out, size_t *out_n);
int register_external_global(const String *key, Value value, void *user);

#endif /* MICA_MODULE_INTERNAL_H */
