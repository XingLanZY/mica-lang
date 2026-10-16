#ifndef MICA_CODEGEN_H
#define MICA_CODEGEN_H

#include <stddef.h>

#include "ast.h"
#include "obj.h"
#include "peephole.h"

typedef struct {
    int line;
    int col;
    char message[256];
} CodegenError;

/* AST → 顶层永久闭包（ObjClosure，内含 proto）。出错返回 NULL 并填充 err。
   v1 编译错误（不支持的语法特性）会明确报"v1 之后支持"。 */
ObjClosure *codegen_program(Gc *gc, PeepholeState *ps, const AstNode *const *stmts, size_t n,
                           CodegenError *err);

/* 编译模块：module_name 为点分模块名（如 "System.IO"）；模块内部声明的全局
   以 "module_name.name" 前缀注册，internal_names 列出这些内部名。
   入口逻辑（顶层语句 或 main()）只对非模块程序生效。 */
ObjClosure *codegen_program_module(Gc *gc, PeepholeState *ps, const AstNode *const *stmts, size_t n,
                                   const char *module_name, const char *const *internal_names,
                                   size_t n_internal, CodegenError *err);

#endif /* MICA_CODEGEN_H */
