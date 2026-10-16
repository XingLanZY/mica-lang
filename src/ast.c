#include "ast.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---------- Arena ---------- */

#define ARENA_ALIGN 16
#define ARENA_BLOCK_SIZE (64 * 1024)

typedef struct ArenaBlock {
    struct ArenaBlock *next;
    size_t used;
    size_t cap;
} ArenaBlock;

struct Arena {
    ArenaBlock *head;
};

Arena *arena_new(void) {
    Arena *a = calloc(1, sizeof(Arena));
    assert(a != NULL);
    return a;
}

void arena_free(Arena *a) {
    if (!a) return;
    ArenaBlock *b = a->head;
    while (b) {
        ArenaBlock *next = b->next;
        free(b);
        b = next;
    }
    free(a);
}

static char *block_data(ArenaBlock *b) {
    uintptr_t p = (uintptr_t)(b + 1);
    p = (p + ARENA_ALIGN - 1) & ~(uintptr_t)(ARENA_ALIGN - 1);
    return (char *)p;
}

void *arena_alloc(Arena *a, size_t size) {
    size = (size + ARENA_ALIGN - 1) & ~(size_t)(ARENA_ALIGN - 1);
    ArenaBlock *b = a->head;
    if (b == NULL || b->used + size > b->cap) {
        /* 单次分配可能超过默认块大小：块容量按需求取大者，避免死循环 */
        size_t block_cap = size > ARENA_BLOCK_SIZE ? size : ARENA_BLOCK_SIZE;
        ArenaBlock *nb = malloc(sizeof(ArenaBlock) + block_cap + ARENA_ALIGN);
        assert(nb != NULL);
        nb->next = a->head;
        nb->used = 0;
        nb->cap = block_cap;
        a->head = nb;
        b = nb;
    }
    char *p = block_data(b) + b->used;
    b->used += size;
    memset(p, 0, size);
    return p;
}

char *arena_copy_str(Arena *a, const char *s, size_t len) {
    char *p = arena_alloc(a, len + 1);
    memcpy(p, s, len);
    p[len] = '\0';
    return p;
}

/* ---------- 节点构造 ---------- */

AstNode *ast_new(Arena *a, AstKind kind, int line, int col) {
    AstNode *n = arena_alloc(a, sizeof(AstNode));
    n->kind = kind;
    n->line = line;
    n->col = col;
    return n;
}

/* ---------- 打印 ---------- */

static void print_raw(const char *s, size_t len) {
    for (size_t i = 0; i < len; i++) {
        unsigned char c = (unsigned char)s[i];
        if (c == '\n') fputs("\\n", stdout);
        else if (c == '\r') fputs("\\r", stdout);
        else if (c == '\t') fputs("\\t", stdout);
        else if (c < 0x20 || c == 0x7f) printf("\\x%02X", c);
        else putchar(c);
    }
}

static void print_indent(int indent) {
    for (int i = 0; i < indent; i++) putchar(' ');
}

static void print_string_node(const AstNode *n) {
    putchar('"');
    for (size_t i = 0; i < n->as.str.n_parts; i++) {
        const AstStrPart *p = &n->as.str.parts[i];
        if (p->is_literal) {
            print_raw(p->text, p->len);
        } else {
            fputs("${", stdout);
            ast_print_expr(p->expr);
            putchar('}');
        }
    }
    putchar('"');
}

static void print_params(const AstParam *params, size_t n, bool has_varargs) {
    putchar('(');
    size_t written = 0;
    for (size_t i = 0; i < n; i++) {
        if (written) fputs(", ", stdout);
        const AstParam *p = &params[i];
        if (p->is_self && p->name != NULL) {
            printf("%s: self", p->name);
        } else {
            fputs(p->name, stdout);
        }
        if (p->has_default) {
            fputs(" = ", stdout);
            ast_print_expr(p->default_expr);
        }
        written++;
    }
    if (has_varargs) {
        if (written) fputs(", ", stdout);
        fputs("...", stdout);
    }
    putchar(')');
}

/* 值块的"值"：末句表达式语句的表达式（`;` 结尾的调用不产值）；否则 NULL。 */
static const AstNode *block_value(const AstNode *block) {
    if (block->as.block.n == 0) return NULL;
    const AstNode *last = block->as.block.stmts[block->as.block.n - 1];
    if (last->kind == AST_EXPR_STMT && !last->as.expr_stmt.no_value) {
        return last->as.expr_stmt.expr;
    }
    return NULL;
}

void ast_print_expr(const AstNode *n) {
    if (!n) {
        fputs("(null)", stdout);
        return;
    }
    switch (n->kind) {        case AST_NUMBER: print_raw(n->as.number.text, n->as.number.len); break;
        case AST_STRING: print_string_node(n); break;
        case AST_NULL: fputs("null", stdout); break;
        case AST_TRUE: fputs("true", stdout); break;
        case AST_FALSE: fputs("false", stdout); break;
        case AST_UNINITIALIZED: fputs("uninitialized", stdout); break;
        case AST_IDENT: fputs(n->as.ident.name, stdout); break;
        case AST_VARARGS: fputs("...", stdout); break;
        case AST_ERROR_VAL:
            fputs("error(", stdout);
            ast_print_expr(n->as.error_val.msg);
            putchar(')');
            break;
        case AST_ARRAY: {
            putchar('[');
            for (size_t i = 0; i < n->as.array.n; i++) {
                if (i) fputs(", ", stdout);
                ast_print_expr(n->as.array.elems[i]);
            }
            putchar(']');
            break;
        }
        case AST_MAP: {
            putchar('{');
            for (size_t i = 0; i < n->as.map.n; i++) {
                if (i) fputs(", ", stdout);
                ast_print_expr(n->as.map.entries[i].key);
                fputs(": ", stdout);
                ast_print_expr(n->as.map.entries[i].value);
            }
            putchar('}');
            break;
        }
        case AST_UNARY:
            printf("(%s", token_kind_name((TokenKind)n->as.unary.op));
            putchar(' ');
            ast_print_expr(n->as.unary.operand);
            putchar(')');
            break;
        case AST_BINARY:
            putchar('(');
            ast_print_expr(n->as.binary.left);
            putchar(' ');
            fputs(token_kind_name((TokenKind)n->as.binary.op), stdout);
            putchar(' ');
            ast_print_expr(n->as.binary.right);
            putchar(')');
            break;
        case AST_TERNARY:
            putchar('(');
            ast_print_expr(n->as.ternary.cond);
            fputs(" ? ", stdout);
            ast_print_expr(n->as.ternary.then_);
            fputs(" : ", stdout);
            ast_print_expr(n->as.ternary.else_);
            putchar(')');
            break;
        case AST_CALL:
            ast_print_expr(n->as.call.callee);
            putchar('(');
            for (size_t i = 0; i < n->as.call.n_args; i++) {
                if (i) fputs(", ", stdout);
                const AstArg *a = &n->as.call.args[i];
                if (a->named) {
                    printf("%s = ", a->name);
                }
                ast_print_expr(a->expr);
            }
            putchar(')');
            break;
        case AST_INDEX:
            ast_print_expr(n->as.index.obj);
            putchar('[');
            ast_print_expr(n->as.index.index);
            putchar(']');
            break;
        case AST_MEMBER:
            ast_print_expr(n->as.member.obj);
            putchar('.');
            fputs(n->as.member.name, stdout);
            break;
        case AST_LAMBDA:
            print_params(n->as.lambda.params, n->as.lambda.n_params, n->as.lambda.has_varargs);
            fputs(" => ", stdout);
            ast_print_expr(n->as.lambda.body);
            break;
        case AST_FUNC_DECL:
            fputs("Func ", stdout);
            fputs(n->as.func.name ? n->as.func.name : "(anon)", stdout);
            print_params(n->as.func.params, n->as.func.n_params, n->as.func.has_varargs);
            if (n->as.func.is_throws) fputs(" throws", stdout);
            break;
        case AST_TRY:
            fputs(token_kind_name((TokenKind)n->as.try_.kind), stdout);
            putchar(' ');
            ast_print_expr(n->as.try_.expr);
            break;
        case AST_OPTIONAL:
            ast_print_expr(n->as.optional.obj);
            fputs("?.", stdout);
            fputs(n->as.optional.name, stdout);
            if (n->as.optional.is_call) fputs("(...)", stdout);
            break;
        case AST_THIS_FUNC:
            fputs("this_func", stdout);
            break;
        case AST_YIELD:
            fputs("yield", stdout);
            if (n->as.yield_.expr) {
                putchar(' ');
                ast_print_expr(n->as.yield_.expr);
            }
            break;
        case AST_DEFER:
            fputs("defer", stdout);
            break;
        case AST_CATCH:
            ast_print_expr(n->as.catch_.expr);
            fputs(" catch ", stdout);
            {
                const CatchHandler *h = &n->as.catch_.handler;
                switch (h->kind) {
                    case CATCH_FALLBACK:
                        ast_print_expr(h->fallback);
                        break;
                    case CATCH_BLOCK: {
                        fputs(h->error_name ? h->error_name : "_", stdout);
                        const AstNode *val = h->fallback ? block_value(h->fallback) : NULL;
                        if (val) {
                            fputs(" { ", stdout);
                            ast_print_expr(val);
                            fputs(" }", stdout);
                        } else {
                            fputs(" { }", stdout);
                        }
                        break;
                    }
                    case CATCH_MATCH:
                        fputs("{ ", stdout);
                        for (size_t i = 0; i < h->n_arms; i++) {
                            if (i) fputs(", ", stdout);
                            const AstMatchArm *arm = &h->arms[i];
                            if (arm->pattern == ARM_WILDCARD) {
                                fputs("_", stdout);
                            } else if (arm->pattern == ARM_BIND && arm->bind) {
                                fputs(arm->bind, stdout);
                            } else {
                                if (arm->set_name) printf("%s.", arm->set_name);
                                fputs(arm->case_name ? arm->case_name : "?", stdout);
                                if (arm->n_payload > 0) {
                                    putchar('(');
                                    for (size_t j = 0; j < arm->n_payload; j++) {
                                        if (j) fputs(", ", stdout);
                                        fputs(arm->payload[j], stdout);
                                    }
                                    putchar(')');
                                }
                            }
                            fputs(" => ", stdout);
                            if (arm->value && arm->value->kind == AST_BLOCK) {
                                const AstNode *bv = block_value(arm->value);
                                if (bv) {
                                    fputs("Block[", stdout);
                                    ast_print_expr(bv);
                                    putchar(']');
                                } else {
                                    printf("Block[%zu stmts]", arm->value->as.block.n);
                                }
                            } else {
                                ast_print_expr(arm->value);
                            }
                        }
                        fputs(" }", stdout);
                        break;
                }
            }
            break;
        case AST_BLOCK_EXPR: {
            const AstNode *val = block_value(n);
            fputs("BlockExpr", stdout);
            if (val) {
                fputs("[", stdout);
                ast_print_expr(val);
                putchar(']');
            } else {
                printf("[%zu stmts]", n->as.block.n);
            }
            break;
        }
        case AST_MATCH:
            fputs("Match ", stdout);
            ast_print_expr(n->as.match_.expr);
            fputs(" { ", stdout);
            for (size_t i = 0; i < n->as.match_.n_arms; i++) {
                if (i) fputs(", ", stdout);
                const AstMatchArm *arm = &n->as.match_.arms[i];
                switch (arm->pattern) {
                    case ARM_LITERAL:
                        ast_print_expr(arm->literal);
                        break;
                    case ARM_BIND:
                        fputs(arm->bind, stdout);
                        break;
                    case ARM_WILDCARD:
                        fputs("_", stdout);
                        break;
                    case ARM_ENUM:
                        if (arm->set_name) printf("%s.", arm->set_name);
                        fputs(arm->case_name, stdout);
                        if (arm->n_payload > 0) {
                            putchar('(');
                            for (size_t j = 0; j < arm->n_payload; j++) {
                                if (j) fputs(", ", stdout);
                                fputs(arm->payload[j], stdout);
                            }
                            putchar(')');
                        }
                        break;
                }
                fputs(" => ", stdout);
                if (arm->value && arm->value->kind == AST_BLOCK) {
                    const AstNode *bv = block_value(arm->value);
                    if (bv) {
                        fputs("Block[", stdout);
                        ast_print_expr(bv);
                        putchar(']');
                    } else {
                        printf("Block[%zu stmts]", arm->value->as.block.n);
                    }
                } else {
                    ast_print_expr(arm->value);
                }
            }
            fputs(" }", stdout);
            break;
        case AST_IF:
            fputs("If ", stdout);
            ast_print_expr(n->as.if_.cond);
            fputs(" {...}", stdout);
            break;
        case AST_WHILE:
            fputs("While ", stdout);
            ast_print_expr(n->as.while_.cond);
            fputs(" {...}", stdout);
            break;
        case AST_FOR_IN:
            fputs("For ", stdout);
            for (size_t i = 0; i < n->as.for_in.n_vars; i++) {
                if (i) fputs(", ", stdout);
                fputs(n->as.for_in.vars[i], stdout);
            }
            fputs(" in ", stdout);
            ast_print_expr(n->as.for_in.iterable);
            fputs(" {...}", stdout);
            break;
        case AST_BLOCK:
            printf("Block(%zu stmts)", n->as.block.n);
            break;
        default:
            printf("<expr kind=%d>", (int)n->kind);
            break;
    }
}

static void print_block_contents(const AstNode *block, int indent) {
    print_indent(indent);
    fputs("Block", stdout);
    if (block->as.block.n == 0) {
        putchar('\n');
        return;
    }
    putchar('\n');
    for (size_t i = 0; i < block->as.block.n; i++) {
        ast_print_stmt(block->as.block.stmts[i], indent + 2);
    }
}

void ast_print_stmt(const AstNode *n, int indent) {
    if (!n) return;
    switch (n->kind) {
        case AST_VAR_DECL: {
            print_indent(indent);
            fputs(n->as.var_decl.is_const ? "Let " : "Var ", stdout);
            for (size_t i = 0; i < n->as.var_decl.n_names; i++) {
                if (i) fputs(", ", stdout);
                fputs(n->as.var_decl.names[i], stdout);
            }
            if (n->as.var_decl.n_values > 0) {
                fputs(" = ", stdout);
                for (size_t i = 0; i < n->as.var_decl.n_values; i++) {
                    if (i) fputs(", ", stdout);
                    ast_print_expr(n->as.var_decl.values[i]);
                }
            }
            putchar('\n');
            break;
        }
        case AST_PROP_DECL:
            print_indent(indent);
            fputs("Prop ", stdout);
            fputs(n->as.prop_decl.name, stdout);
            fputs(" => ", stdout);
            ast_print_expr(n->as.prop_decl.body);
            putchar('\n');
            break;
        case AST_ASSIGN: {
            print_indent(indent);
            for (size_t i = 0; i < n->as.assign.n; i++) {
                if (i) fputs(", ", stdout);
                ast_print_expr(n->as.assign.targets[i]);
            }
            putchar(' ');
            fputs(token_kind_name((TokenKind)n->as.assign.op), stdout);
            putchar(' ');
            for (size_t i = 0; i < n->as.assign.n_values; i++) {
                if (i) fputs(", ", stdout);
                ast_print_expr(n->as.assign.values[i]);
            }
            putchar('\n');
            break;
        }
        case AST_EXPR_STMT:
            print_indent(indent);
            ast_print_expr(n->as.expr_stmt.expr);
            putchar('\n');
            break;
        case AST_DEFER:
            print_indent(indent);
            fputs("Defer\n", stdout);
            print_block_contents(n->as.defer_.block, indent + 2);
            break;
        case AST_TRY_BLOCK:
            print_indent(indent);
            fputs(token_kind_name((TokenKind)n->as.try_block.kind), stdout);
            fputs("Block\n", stdout);
            print_block_contents(n->as.try_block.body, indent + 2);
            break;
        case AST_DO_CATCH:
            print_indent(indent);
            fputs("DoCatch\n", stdout);
            print_block_contents(n->as.do_catch.body, indent + 2);
            print_indent(indent);
            fputs("Catch\n", stdout);
            for (size_t i = 0; i < n->as.do_catch.handler.n_arms; i++) {
                print_indent(indent + 2);
                fputs("arm => ", stdout);
                ast_print_expr(n->as.do_catch.handler.arms[i].value);
                putchar('\n');
            }
            break;
        case AST_BLOCK:
            print_block_contents(n, indent);
            break;
        case AST_BLOCK_EXPR:
            print_indent(indent);
            fputs("BlockExpr\n", stdout);
            print_block_contents(n, indent + 2);
            break;
        case AST_IF: {
            print_indent(indent);
            fputs("If ", stdout);
            ast_print_expr(n->as.if_.cond);
            putchar('\n');
            print_block_contents(n->as.if_.then_, indent + 2);
            for (size_t i = 0; i < n->as.if_.n_elif; i++) {
                print_indent(indent);
                fputs("Elif ", stdout);
                ast_print_expr(n->as.if_.elif_conds[i]);
                putchar('\n');
                print_block_contents(n->as.if_.elif_bodies[i], indent + 2);
            }
            if (n->as.if_.else_) {
                print_indent(indent);
                fputs("Else\n", stdout);
                print_block_contents(n->as.if_.else_, indent + 2);
            }
            break;
        }
        case AST_WHILE:
            print_indent(indent);
            fputs("While ", stdout);
            ast_print_expr(n->as.while_.cond);
            putchar('\n');
            print_block_contents(n->as.while_.body, indent + 2);
            break;
        case AST_FOR_IN: {
            print_indent(indent);
            fputs("For ", stdout);
            for (size_t i = 0; i < n->as.for_in.n_vars; i++) {
                if (i) fputs(", ", stdout);
                fputs(n->as.for_in.vars[i], stdout);
            }
            fputs(" in ", stdout);
            ast_print_expr(n->as.for_in.iterable);
            if (n->as.for_in.step) {
                fputs(" step ", stdout);
                ast_print_expr(n->as.for_in.step);
            }
            putchar('\n');
            print_block_contents(n->as.for_in.body, indent + 2);
            break;
        }
        case AST_BREAK:
            print_indent(indent);
            fputs("Break\n", stdout);
            break;
        case AST_CONTINUE:
            print_indent(indent);
            fputs("Continue\n", stdout);
            break;
        case AST_RETURN: {
            print_indent(indent);
            fputs("Return", stdout);
            if (n->as.return_.n > 0) {
                putchar(' ');
                for (size_t i = 0; i < n->as.return_.n; i++) {
                    if (i) fputs(", ", stdout);
                    ast_print_expr(n->as.return_.values[i]);
                }
            }
            putchar('\n');
            break;
        }
        case AST_THROW:
            print_indent(indent);
            fputs("Throw ", stdout);
            ast_print_expr(n->as.throw_.expr);
            putchar('\n');
            break;
        case AST_FUNC_DECL: {
            print_indent(indent);
            if (n->as.func.is_static) fputs("Static ", stdout);
            fputs("Func ", stdout);
            fputs(n->as.func.name ? n->as.func.name : "(anon)", stdout);
            print_params(n->as.func.params, n->as.func.n_params, n->as.func.has_varargs);
            if (n->as.func.is_throws) fputs(" throws", stdout);
            putchar('\n');
            if (n->as.func.body && n->as.func.body->kind == AST_BLOCK) {
                print_block_contents(n->as.func.body, indent + 2);
            } else {
                /* 计算属性 getter（let r => expr）：body 是表达式 */
                print_indent(indent + 2);
                fputs("=> ", stdout);
                ast_print_expr(n->as.func.body);
                putchar('\n');
            }
            break;
        }
        case AST_CLASS_DECL: {
            print_indent(indent);
            fputs("Class ", stdout);
            fputs(n->as.class_.name, stdout);
            if (n->as.class_.parent) {
                fputs(" : ", stdout);
                fputs(n->as.class_.parent, stdout);
            }
            putchar('\n');
            for (size_t i = 0; i < n->as.class_.n_members; i++) {
                ast_print_stmt(n->as.class_.members[i], indent + 2);
            }
            break;
        }
        case AST_ERROR_SET_DECL: {
            print_indent(indent);
            fputs("ErrorSet ", stdout);
            fputs(n->as.error_set.name, stdout);
            fputs(" { ", stdout);
            for (size_t i = 0; i < n->as.error_set.n_cases; i++) {
                if (i) fputs(", ", stdout);
                fputs(n->as.error_set.cases[i].name, stdout);
                if (n->as.error_set.cases[i].n_payload > 0) {
                    putchar('(');
                    for (size_t j = 0; j < n->as.error_set.cases[i].n_payload; j++) {
                        if (j) fputs(", ", stdout);
                        fputs(n->as.error_set.cases[i].payload[j], stdout);
                    }
                    putchar(')');
                }
            }
            fputs(" }\n", stdout);
            break;
        }
        case AST_ENUM_DECL: {
            print_indent(indent);
            fputs("Enum ", stdout);
            fputs(n->as.enum_.name, stdout);
            fputs(" { ", stdout);
            for (size_t i = 0; i < n->as.enum_.n_members; i++) {
                if (i) fputs(", ", stdout);
                fputs(n->as.enum_.members[i].name, stdout);
                if (n->as.enum_.members[i].n_payload > 0) {
                    putchar('(');
                    for (size_t j = 0; j < n->as.enum_.members[i].n_payload; j++) {
                        if (j) fputs(", ", stdout);
                        fputs(n->as.enum_.members[i].payload[j], stdout);
                    }
                    putchar(')');
                }
                if (n->as.enum_.members[i].value) {
                    fputs(" = ", stdout);
                    ast_print_expr(n->as.enum_.members[i].value);
                }
            }
            fputs(" }\n", stdout);
            break;
        }
        case AST_IMPORT:
            print_indent(indent);
            fputs("Import ", stdout);
            fputs(n->as.import.name, stdout);
            if (n->as.import.n_names > 0) {
                fputs(".{ ", stdout);
                for (size_t i = 0; i < n->as.import.n_names; i++) {
                    if (i) fputs(", ", stdout);
                    fputs(n->as.import.names[i], stdout);
                }
                fputs(" }", stdout);
            }
            putchar('\n');
            break;
        case AST_EXPORT_MODULE:
            print_indent(indent);
            fputs("ExportModule ", stdout);
            fputs(n->as.export_module.name, stdout);
            putchar('\n');
            break;
        case AST_EXPORT:
            print_indent(indent);
            fputs("Export\n", stdout);
            ast_print_stmt(n->as.export_.decl, indent + 2);
            break;
        default:
            print_indent(indent);
            printf("<stmt kind=%d>\n", (int)n->kind);
            break;
    }
}

void ast_print_program(const AstNode *const *stmts, size_t n) {
    fputs("Program\n", stdout);
    for (size_t i = 0; i < n; i++) {
        ast_print_stmt(stmts[i], 2);
    }
}
