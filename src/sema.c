#include "sema.h"

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "map.h"

#define MAX_SEMA_ERRORS 50

typedef enum {
    SYM_VAR,
    SYM_FUNC,
    SYM_CLASS,
    SYM_ERR_SET,
    SYM_ENUM,
    SYM_MODULE,
} SymKind;

typedef struct {
    SymKind kind;
    const AstNode *decl;           /* 声明节点 */
    const AstNode *inferred_class; /* SYM_VAR：初始化为类构造时的类节点 */
} Symbol;

typedef struct Scope {
    struct Scope *parent;
    Map *symbols;
} Scope;

typedef struct MapNode {
    Map *map;
    struct MapNode *next;
} MapNode;

struct Sema {
    Arena *arena;
    ParserError errors[MAX_SEMA_ERRORS];
    size_t n_errors;
    bool fatal;
    Scope *global;
    MapNode *maps;
    Symbol builtin_sym;
    int func_depth;
    int loop_depth;
    bool cur_in_method;   /* 当前是否在方法内（lambda 继承此上下文） */
    const AstNode *cur_class_node; /* 当前方法所属类节点（self 类推断） */
    bool cur_has_varargs;
    const char **external;   /* 外部全局名（宿主注入/先前运行产生） */
    size_t n_external, cap_external;
};

/* ---------- 内置名 ---------- */

static bool is_builtin(const char *name) {
    static const char *const k_builtins[] = {
        "print", "typeof", "len", "int", "float", "string", "bool",
        "File", "System", "Math", "IO", "Native", "Error", "Input",
        "assembly",
    };
    for (size_t i = 0; i < sizeof(k_builtins) / sizeof(k_builtins[0]); i++) {
        if (strcmp(name, k_builtins[i]) == 0) return true;
    }
    return false;
}

/* ---------- 错误 ---------- */

static void sema_error_at(Sema *s, int line, int col, const char *fmt, ...) {
    if (s->n_errors >= MAX_SEMA_ERRORS) {
        s->fatal = true;
        return;
    }
    ParserError *e = &s->errors[s->n_errors++];
    e->line = line;
    e->col = col;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(e->message, sizeof(e->message), fmt, ap);
    va_end(ap);
}

/* ---------- 作用域 ---------- */

static Map *make_map(Sema *s) {
    Map *m = map_new(sizeof(Symbol), NULL);
    MapNode *node = malloc(sizeof(MapNode));
    assert(node != NULL);
    node->map = m;
    node->next = s->maps;
    s->maps = node;
    return m;
}

static Scope *new_scope(Sema *s, Scope *parent) {
    Scope *sc = arena_alloc(s->arena, sizeof(Scope));
    sc->parent = parent;
    sc->symbols = make_map(s);
    return sc;
}

static const Symbol *lookup(Scope *scope, const char *name) {
    for (Scope *sc = scope; sc; sc = sc->parent) {
        const Symbol *sym = map_find(sc->symbols, name);
        if (sym) return sym;
    }
    return NULL;
}

static const Symbol *lookup_ident(Sema *s, Scope *scope, const char *name) {
    const Symbol *sym = lookup(scope, name);
    if (sym) return sym;
    if (is_builtin(name)) return &s->builtin_sym;
    return NULL;
}

static void register_name(Sema *s, Scope *scope, const char *name, SymKind kind,
                          const AstNode *decl, int line, int col) {
    if (map_has(scope->symbols, name)) {
        sema_error_at(s, line, col, "duplicate declaration of '%s'", name);
        return;
    }
    Symbol sym;
    sym.kind = kind;
    sym.decl = decl;
    sym.inferred_class = NULL;
    map_push_v(scope->symbols, name, &sym);
}

/* 拷贝点分名的首段（如 "System.Math" -> "System"）。 */
static const char *first_segment(Sema *s, const char *dotted) {
    if (!dotted) return NULL;
    const char *dot = strchr(dotted, '.');
    size_t len = dot ? (size_t)(dot - dotted) : strlen(dotted);
    if (len == 0) return NULL;
    return arena_copy_str(s->arena, dotted, len);
}

/* 拷贝点分名的末段（如 "System.Math" -> "Math"），供短名导入绑定。 */
static const char *last_segment(Sema *s, const char *dotted) {
    if (!dotted) return NULL;
    const char *dot = strrchr(dotted, '.');
    const char *start = dot ? dot + 1 : dotted;
    return arena_copy_str(s->arena, start, strlen(start));
}

/* ---------- 类成员查找（含继承） ---------- */

static const AstNode *find_member(Sema *s, Scope *scope, const AstNode *class_node,
                                  const char *name, bool want_static) {
    for (size_t i = 0; i < class_node->as.class_.n_members; i++) {
        const AstNode *m = class_node->as.class_.members[i];
        if (m->kind == AST_FUNC_DECL && m->as.func.is_static == want_static &&
            m->as.func.name && strcmp(m->as.func.name, name) == 0) {
            return m;
        }
    }
    if (class_node->as.class_.parent) {
        const Symbol *psym = lookup(scope, class_node->as.class_.parent);
        if (psym && psym->kind == SYM_CLASS) {
            return find_member(s, scope, psym->decl, name, want_static);
        }
    }
    return NULL;
}

/* ---------- 调用目标解析与命名参数校验 ---------- */

typedef struct {
    const AstParam *params;
    size_t n_params;
    bool skip_self;   /* 方法：self 不参与命名参数匹配 */
    const char *desc;
    bool found;
} CalleeInfo;

static const char *func_desc(Sema *s, const AstNode *func) {
    if (func->as.func.name) {
        char *buf = arena_alloc(s->arena, 64);
        snprintf(buf, 64, "function '%s'", func->as.func.name);
        return buf;
    }
    return "anonymous function";
}

static const char *member_desc(Sema *s, const AstNode *class_node, const char *name,
                               const char *kind) {
    char *buf = arena_alloc(s->arena, 80);
    snprintf(buf, 80, "%s '%s' of '%s'", kind, name, class_node->as.class_.name);
    return buf;
}

static void resolve_callee(Sema *s, Scope *scope, const AstNode *callee, CalleeInfo *out) {
    memset(out, 0, sizeof(*out));
    if (callee->kind == AST_IDENT) {
        const char *name = callee->as.ident.name;
        if (strcmp(name, "super") == 0) return; /* super 调用无法静态校验 */
        const Symbol *sym = lookup_ident(s, scope, name);
        if (!sym || sym == &s->builtin_sym) return; /* 内置名无法静态校验参数 */
        if (sym->kind == SYM_FUNC) {
            const AstNode *f = sym->decl;
            out->params = f->as.func.params;
            out->n_params = f->as.func.n_params;
            out->skip_self = false;
            out->desc = func_desc(s, f);
            out->found = true;
        } else if (sym->kind == SYM_CLASS) {
            const AstNode *init = find_member(s, scope, sym->decl, "init", false);
            if (init) {
                out->params = init->as.func.params;
                out->n_params = init->as.func.n_params;
                out->skip_self = true;
                out->desc = member_desc(s, sym->decl, "init", "constructor");
                out->found = true;
            }
        }
        return;
    }
    if (callee->kind == AST_MEMBER && callee->as.member.obj->kind == AST_IDENT) {
        const Symbol *base = lookup_ident(s, scope, callee->as.member.obj->as.ident.name);
        if (!base || base == &s->builtin_sym) return;
        if (base->kind == SYM_CLASS) {
            const AstNode *m = find_member(s, scope, base->decl, callee->as.member.name, true);
            if (m) {
                out->params = m->as.func.params;
                out->n_params = m->as.func.n_params;
                out->skip_self = false;
                out->desc = member_desc(s, base->decl, callee->as.member.name, "static method");
                out->found = true;
            }
        } else if (base->kind == SYM_VAR && base->inferred_class) {
            const AstNode *m = find_member(s, scope, base->inferred_class, callee->as.member.name,
                                           false);
            if (m) {
                out->params = m->as.func.params;
                out->n_params = m->as.func.n_params;
                out->skip_self = true;
                out->desc = member_desc(s, base->inferred_class, callee->as.member.name, "method");
                out->found = true;
            }
        }
    }
}

static void check_named_args(Sema *s, int line, int col, const CalleeInfo *info,
                             const AstArg *args, size_t n_args) {
    if (!info->found) return;
    const size_t first = info->skip_self && info->n_params > 0 ? 1 : 0;
    for (size_t i = 0; i < n_args; i++) {
        const AstArg *a = &args[i];
        if (!a->named) continue;
        bool ok = false;
        for (size_t j = first; j < info->n_params; j++) {
            if (strcmp(info->params[j].name, a->name) == 0) {
                ok = true;
                break;
            }
        }
        if (!ok) {
            sema_error_at(s, line, col, "unknown named argument '%s' for %s", a->name, info->desc);
        }
    }
}

/* ---------- 前向声明 ---------- */

static void sema_stmt(Sema *s, Scope *scope, const AstNode *n, bool top_level);
static void sema_expr(Sema *s, Scope *scope, const AstNode *n);
static void sema_block_contents(Sema *s, Scope *scope, const AstNode *block, bool is_value_block);

/* 可作为独立语句的表达式：调用/构造（有副作用）与 get 块表达式。其余裸表达式语句被禁止。 */
static bool is_stmt_expr(const AstNode *e) {
    if (!e) return false;
    return e->kind == AST_CALL || e->kind == AST_ERROR_VAL || e->kind == AST_YIELD ||
           e->kind == AST_BLOCK_EXPR || e->kind == AST_TRY || e->kind == AST_CATCH;
}

/* ---------- 函数体（作用域 + 上下文） ---------- */

/* 树中是否含 yield（不深入嵌套 func/lambda/defer——它们各自独立协程函数）。 */
static bool ast_contains_yield(const AstNode *n) {
    if (!n) return false;
    switch (n->kind) {
        case AST_YIELD: return true;
        case AST_FUNC_DECL:
        case AST_LAMBDA:
        case AST_DEFER:
        case AST_PROP_DECL:
            return false;
        case AST_BLOCK:
        case AST_BLOCK_EXPR:
            for (size_t i = 0; i < n->as.block.n; i++)
                if (ast_contains_yield(n->as.block.stmts[i])) return true;
            return false;
        case AST_IF:
            if (ast_contains_yield(n->as.if_.cond)) return true;
            if (n->as.if_.then_ && ast_contains_yield(n->as.if_.then_)) return true;
            for (size_t i = 0; i < n->as.if_.n_elif; i++) {
                if (ast_contains_yield(n->as.if_.elif_conds[i])) return true;
                if (n->as.if_.elif_bodies[i] && ast_contains_yield(n->as.if_.elif_bodies[i])) return true;
            }
            return n->as.if_.else_ && ast_contains_yield(n->as.if_.else_);
        case AST_WHILE:
            return ast_contains_yield(n->as.while_.cond) || ast_contains_yield(n->as.while_.body);
        case AST_FOR_IN:
            if (ast_contains_yield(n->as.for_in.iterable)) return true;
            if (n->as.for_in.step && ast_contains_yield(n->as.for_in.step)) return true;
            return ast_contains_yield(n->as.for_in.body);
        case AST_VAR_DECL:
            for (size_t i = 0; i < n->as.var_decl.n_values; i++)
                if (ast_contains_yield(n->as.var_decl.values[i])) return true;
            return false;
        case AST_ASSIGN:
            for (size_t i = 0; i < n->as.assign.n; i++)
                if (ast_contains_yield(n->as.assign.targets[i])) return true;
            for (size_t i = 0; i < n->as.assign.n_values; i++)
                if (ast_contains_yield(n->as.assign.values[i])) return true;
            return false;
        case AST_EXPR_STMT: return ast_contains_yield(n->as.expr_stmt.expr);
        case AST_RETURN:
            for (size_t i = 0; i < n->as.return_.n; i++)
                if (ast_contains_yield(n->as.return_.values[i])) return true;
            return false;
        case AST_UNARY: return ast_contains_yield(n->as.unary.operand);
        case AST_BINARY:
            return ast_contains_yield(n->as.binary.left) || ast_contains_yield(n->as.binary.right);
        case AST_TERNARY:
            return ast_contains_yield(n->as.ternary.cond) || ast_contains_yield(n->as.ternary.then_) ||
                   ast_contains_yield(n->as.ternary.else_);
        case AST_CALL:
            if (ast_contains_yield(n->as.call.callee)) return true;
            for (size_t i = 0; i < n->as.call.n_args; i++)
                if (ast_contains_yield(n->as.call.args[i].expr)) return true;
            return false;
        case AST_ARRAY:
            for (size_t i = 0; i < n->as.array.n; i++)
                if (ast_contains_yield(n->as.array.elems[i])) return true;
            return false;
        case AST_MAP:
            for (size_t i = 0; i < n->as.map.n; i++) {
                if (ast_contains_yield(n->as.map.entries[i].key)) return true;
                if (ast_contains_yield(n->as.map.entries[i].value)) return true;
            }
            return false;
        case AST_INDEX:
            return ast_contains_yield(n->as.index.obj) || ast_contains_yield(n->as.index.index);
        case AST_MEMBER: return ast_contains_yield(n->as.member.obj);
        case AST_TRY: return ast_contains_yield(n->as.try_.expr);
        case AST_TRY_BLOCK: return ast_contains_yield(n->as.try_block.body);
        case AST_OPTIONAL:
            if (ast_contains_yield(n->as.optional.obj)) return true;
            for (size_t i = 0; i < n->as.optional.n_args; i++)
                if (ast_contains_yield(n->as.optional.args[i].expr)) return true;
            return false;
        case AST_DO_CATCH: {
            if (ast_contains_yield(n->as.do_catch.body)) return true;
            const CatchHandler *h = &n->as.do_catch.handler;
            if (h->kind == CATCH_FALLBACK || h->kind == CATCH_BLOCK)
                return ast_contains_yield(h->fallback);
            for (size_t i = 0; i < h->n_arms; i++)
                if (ast_contains_yield(h->arms[i].value)) return true;
            return false;
        }
        case AST_CATCH:
            if (ast_contains_yield(n->as.catch_.expr)) return true;
            if (n->as.catch_.handler.kind == CATCH_FALLBACK ||
                n->as.catch_.handler.kind == CATCH_BLOCK) {
                return ast_contains_yield(n->as.catch_.handler.fallback);
            }
            for (size_t i = 0; i < n->as.catch_.handler.n_arms; i++)
                if (ast_contains_yield(n->as.catch_.handler.arms[i].value)) return true;
            return false;
        case AST_MATCH:
            if (ast_contains_yield(n->as.match_.expr)) return true;
            for (size_t i = 0; i < n->as.match_.n_arms; i++)
                if (ast_contains_yield(n->as.match_.arms[i].value)) return true;
            return false;
        default:
            return false;
    }
}

static void sema_func_common(Sema *s, Scope *scope, const AstParam *params, size_t n_params,
                             bool has_varargs, const AstNode *body, int line, int col) {
    Scope *fs = new_scope(s, scope);
    for (size_t i = 0; i < n_params; i++) {
        register_name(s, fs, params[i].name, SYM_VAR, NULL, line, col);
        if (params[i].has_default) sema_expr(s, fs, params[i].default_expr);
    }
    /* self 类推断：实例方法首参 self 的 inferred_class = 所在类，使 self.method(x=1) 可解析 */
    if (s->cur_in_method && n_params > 0 && s->cur_class_node) {
        Symbol *self_sym = (Symbol *)map_find(fs->symbols, params[0].name);
        if (self_sym) self_sym->inferred_class = s->cur_class_node;
    }
    const bool saved_varargs = s->cur_has_varargs;
    const int saved_func = s->func_depth;
    s->cur_has_varargs = has_varargs;
    s->func_depth++;
    if (body && body->kind == AST_BLOCK) {
        sema_block_contents(s, fs, body, false); /* 函数体块不产值 */
    } else {
        sema_expr(s, fs, body);
    }
    s->func_depth = saved_func;
    s->cur_has_varargs = saved_varargs;
}

static void sema_func_body(Sema *s, Scope *scope, const AstNode *func) {
    sema_func_common(s, scope, func->as.func.params, func->as.func.n_params,
                     func->as.func.has_varargs, func->as.func.body, func->line, func->col);
}

static void sema_block_contents(Sema *s, Scope *scope, const AstNode *block, bool is_value_block) {
    if (!block) return;
    const size_t n = block->as.block.n;
    for (size_t i = 0; i < n; i++) {
        const AstNode *st = block->as.block.stmts[i];
        const bool is_value_pos = is_value_block && (i == n - 1);
        if (is_value_pos) {
            /* 值位置：末句表达式/控制流作为块的值（`;` 结尾的调用语句由 codegen 决定不产值） */
            if (st->kind == AST_EXPR_STMT) {
                sema_expr(s, scope, st->as.expr_stmt.expr);
            } else if (st->kind == AST_IF || st->kind == AST_WHILE || st->kind == AST_FOR_IN ||
                       st->kind == AST_BLOCK_EXPR) {
                sema_expr(s, scope, st);
            } else {
                sema_stmt(s, scope, st, false);
            }
        } else {
            sema_stmt(s, scope, st, false);
        }
        if (s->fatal) return;
    }
}

/* ---------- 控制流（as_expr 决定分支块是否产值） ---------- */

static void sema_if(Sema *s, Scope *scope, const AstNode *n, bool as_expr) {
    sema_expr(s, scope, n->as.if_.cond);
    Scope *ts = new_scope(s, scope);
    sema_block_contents(s, ts, n->as.if_.then_, as_expr);
    for (size_t i = 0; i < n->as.if_.n_elif; i++) {
        sema_expr(s, scope, n->as.if_.elif_conds[i]);
        Scope *es = new_scope(s, scope);
        sema_block_contents(s, es, n->as.if_.elif_bodies[i], as_expr);
    }
    if (n->as.if_.else_) {
        Scope *els = new_scope(s, scope);
        sema_block_contents(s, els, n->as.if_.else_, as_expr);
    }
}

static void sema_while(Sema *s, Scope *scope, const AstNode *n, bool as_expr) {
    sema_expr(s, scope, n->as.while_.cond);
    s->loop_depth++;
    Scope *bs = new_scope(s, scope);
    sema_block_contents(s, bs, n->as.while_.body, as_expr);
    s->loop_depth--;
}

static void sema_for(Sema *s, Scope *scope, const AstNode *n, bool as_expr) {
    Scope *fs = new_scope(s, scope);
    for (size_t i = 0; i < n->as.for_in.n_vars; i++) {
        register_name(s, fs, n->as.for_in.vars[i], SYM_VAR, n, n->line, n->col);
    }
    sema_expr(s, fs, n->as.for_in.iterable);
    if (n->as.for_in.step) sema_expr(s, fs, n->as.for_in.step);
    s->loop_depth++;
    Scope *bs = new_scope(s, fs);
    sema_block_contents(s, bs, n->as.for_in.body, as_expr);
    s->loop_depth--;
}

/* ---------- 表达式 ---------- */

/* is 右操作数允许的内置类型名（类名与枚举 case 另行校验）。 */
static bool is_builtin_type_name(const char *name) {
    static const char *const names[] = {
        "number", "int", "float", "bool", "string", "array", "map",
        "function", "error", "coroutine", "enum", "null", "object",
    };
    for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
        if (strcmp(names[i], name) == 0) return true;
    }
    return false;
}

static void sema_expr(Sema *s, Scope *scope, const AstNode *n) {
    if (!n || s->fatal) return;
    switch (n->kind) {
        case AST_NUMBER:
        case AST_STRING:
        case AST_NULL:
        case AST_TRUE:
        case AST_FALSE:
        case AST_UNINITIALIZED:
            break;
        case AST_ERROR_VAL:
            sema_expr(s, scope, n->as.error_val.msg);
            break;
        case AST_IDENT: {
            const char *name = n->as.ident.name;
            if (strcmp(name, "self") == 0) {
                /* self 必须是方法参数（或 lambda 捕获的方法参数）；按作用域解析 */
                if (!lookup(scope, "self"))
                    sema_error_at(s, n->line, n->col, "self is only available inside methods");
                break;
            }
            if (strcmp(name, "super") == 0) {
                if (!s->cur_in_method)
                    sema_error_at(s, n->line, n->col, "super is only available inside methods");
                break;
            }
            if (strcmp(name, "coroutine") == 0) {
                /* coroutine 是协程命名空间（coroutine.create/resume/...），非普通变量 */
                break;
            }
            if (!lookup_ident(s, scope, name)) {
                sema_error_at(s, n->line, n->col, "undefined variable '%s'", name);
            }
            break;
        }
        case AST_VARARGS:
            if (!s->cur_has_varargs)
                sema_error_at(s, n->line, n->col, "varargs '...' used outside a vararg function");
            break;
        case AST_ARRAY:
            for (size_t i = 0; i < n->as.array.n; i++) sema_expr(s, scope, n->as.array.elems[i]);
            break;
        case AST_MAP:
            for (size_t i = 0; i < n->as.map.n; i++) {
                /* 裸标识符键是字符串键（{k: 1}），非变量求值 */
                if (n->as.map.entries[i].key->kind != AST_IDENT) {
                    sema_expr(s, scope, n->as.map.entries[i].key);
                }
                sema_expr(s, scope, n->as.map.entries[i].value);
            }
            break;
        case AST_UNARY:
            sema_expr(s, scope, n->as.unary.operand);
            break;
        case AST_BINARY:
            sema_expr(s, scope, n->as.binary.left);
            if (n->as.binary.op == TOK_IS) {
                /* is 的右操作数是类型引用，不是普通表达式 */
                const AstNode *r = n->as.binary.right;
                if (r->kind == AST_IDENT) {
                    if (is_builtin_type_name(r->as.ident.name)) break;
                    const Symbol *sym = lookup_ident(s, scope, r->as.ident.name);
                    if (sym && sym->kind == SYM_CLASS) break;
                    if (sym && sym->kind == SYM_ENUM) {
                        sema_error_at(s, r->line, r->col,
                                      "enum type '%s' is not directly usable with 'is'; use 'is %s.Case'",
                                      r->as.ident.name, r->as.ident.name);
                        break;
                    }
                    sema_error_at(s, r->line, r->col, "'%s' is not a type name", r->as.ident.name);
                } else if (r->kind == AST_MEMBER) {
                    /* 枚举 case：Color.Red */
                    sema_expr(s, scope, r->as.member.obj);
                } else {
                    sema_error_at(s, r->line, r->col,
                                  "right operand of 'is' must be a type name or enum case");
                }
                break;
            }
            sema_expr(s, scope, n->as.binary.right);
            break;
        case AST_TERNARY:
            sema_expr(s, scope, n->as.ternary.cond);
            sema_expr(s, scope, n->as.ternary.then_);
            sema_expr(s, scope, n->as.ternary.else_);
            break;
        case AST_CALL: {
            sema_expr(s, scope, n->as.call.callee);
            CalleeInfo info;
            resolve_callee(s, scope, n->as.call.callee, &info);
            check_named_args(s, n->line, n->col, &info, n->as.call.args, n->as.call.n_args);
            for (size_t i = 0; i < n->as.call.n_args; i++) {
                sema_expr(s, scope, n->as.call.args[i].expr);
            }
            break;
        }
        case AST_INDEX:
            sema_expr(s, scope, n->as.index.obj);
            sema_expr(s, scope, n->as.index.index);
            break;
        case AST_MEMBER:
            sema_expr(s, scope, n->as.member.obj);
            break;
        case AST_LAMBDA:
            sema_func_common(s, scope, n->as.lambda.params, n->as.lambda.n_params,
                             n->as.lambda.has_varargs, n->as.lambda.body, n->line, n->col);
            break;
        case AST_THIS_FUNC:
            break;
        case AST_YIELD:
            if (n->as.yield_.expr) sema_expr(s, scope, n->as.yield_.expr);
            break;
        case AST_TRY:
            sema_expr(s, scope, n->as.try_.expr);
            break;
        case AST_TRY_BLOCK: {
            Scope *bs = new_scope(s, scope);
            sema_block_contents(s, bs, n->as.try_block.body, true);
            break;
        }
        case AST_OPTIONAL:
            sema_expr(s, scope, n->as.optional.obj);
            for (size_t i = 0; i < n->as.optional.n_args; i++)
                sema_expr(s, scope, n->as.optional.args[i].expr);
            break;
        case AST_DO_CATCH: {
            Scope *bs = new_scope(s, scope);
            sema_block_contents(s, bs, n->as.do_catch.body, true);
            const CatchHandler *h = &n->as.do_catch.handler;
            if (h->kind == CATCH_FALLBACK) {
                sema_expr(s, scope, h->fallback);
            } else if (h->kind == CATCH_BLOCK) {
                Scope *cs = new_scope(s, scope);
                if (h->error_name) {
                    Symbol sym = {SYM_VAR, NULL, NULL};
                    map_push_v(cs->symbols, h->error_name, &sym);
                }
                sema_block_contents(s, cs, h->fallback, true);
            } else { /* CATCH_MATCH */
                for (size_t i = 0; i < h->n_arms; i++) {
                    Scope *as = h->arms[i].n_payload > 0 ? new_scope(s, scope) : scope;
                    if (h->arms[i].n_payload > 0) {
                        for (size_t j = 0; j < h->arms[i].n_payload; j++) {
                            Symbol sym = {SYM_VAR, NULL, NULL};
                            map_push_v(as->symbols, h->arms[i].payload[j], &sym);
                        }
                    }
                    sema_expr(s, as, h->arms[i].value);
                }
            }
            break;
        }
        case AST_CATCH: {
            sema_expr(s, scope, n->as.catch_.expr);
            const CatchHandler *h = &n->as.catch_.handler;
            if (h->kind == CATCH_FALLBACK) {
                sema_expr(s, scope, h->fallback);
            } else if (h->kind == CATCH_BLOCK) {
                Scope *cs = new_scope(s, scope);
                if (h->error_name) {
                    Symbol sym = {SYM_VAR, NULL, NULL};
                    map_push_v(cs->symbols, h->error_name, &sym);
                }
                sema_block_contents(s, cs, h->fallback, true);
            } else { /* CATCH_MATCH */
                for (size_t i = 0; i < h->n_arms; i++) {
                    Scope *as = h->arms[i].n_payload > 0 ? new_scope(s, scope) : scope;
                    if (h->arms[i].n_payload > 0) {
                        for (size_t j = 0; j < h->arms[i].n_payload; j++) {
                            Symbol sym = {SYM_VAR, NULL, NULL};
                            map_push_v(as->symbols, h->arms[i].payload[j], &sym);
                        }
                    }
                    sema_expr(s, as, h->arms[i].value);
                }
            }
            break;
        }
        case AST_MATCH: {
            sema_expr(s, scope, n->as.match_.expr);
            for (size_t i = 0; i < n->as.match_.n_arms; i++) {
                const AstMatchArm *arm = &n->as.match_.arms[i];
                const bool binds = (arm->pattern == ARM_BIND || arm->pattern == ARM_ENUM) &&
                                   (arm->bind != NULL || arm->n_payload > 0);
                Scope *as = binds ? new_scope(s, scope) : scope;
                if (arm->pattern == ARM_LITERAL) {
                    sema_expr(s, scope, arm->literal);
                }
                if (arm->pattern == ARM_BIND && arm->bind) {
                    Symbol sym = {SYM_VAR, NULL, NULL};
                    map_push_v(as->symbols, arm->bind, &sym);
                }
                for (size_t j = 0; j < arm->n_payload; j++) {
                    Symbol sym = {SYM_VAR, NULL, NULL};
                    map_push_v(as->symbols, arm->payload[j], &sym);
                }
                if (arm->value && arm->value->kind == AST_BLOCK) {
                    sema_block_contents(s, as, arm->value, true);
                } else {
                    sema_expr(s, as, arm->value);
                }
            }
            break;
        }
        case AST_BLOCK_EXPR: {
            Scope *bs = new_scope(s, scope);
            sema_block_contents(s, bs, n, true);
            break;
        }
        case AST_IF:
            sema_if(s, scope, n, true); /* 表达式上下文：分支产值 */
            break;
        case AST_WHILE:
            sema_while(s, scope, n, true);
            break;
        case AST_FOR_IN:
            sema_for(s, scope, n, true);
            break;
        case AST_FUNC_DECL:
            sema_stmt(s, scope, n, false);
            break;
        default:
            break;
    }
}

/* ---------- 语句 ---------- */

static void sema_stmt(Sema *s, Scope *scope, const AstNode *n, bool top_level) {
    if (!n || s->fatal) return;
    switch (n->kind) {
        case AST_PROP_DECL: {
            /* 只读计算属性：注册名、校验体（禁 yield，函数体语义） */
            if (!top_level || !map_find(scope->symbols, n->as.prop_decl.name)) {
                register_name(s, scope, n->as.prop_decl.name, SYM_VAR, n, n->line, n->col);
            }
            if (ast_contains_yield(n->as.prop_decl.body)) {
                sema_error_at(s, n->line, n->col,
                              "computed property cannot contain 'yield'");
            }
            sema_func_common(s, scope, NULL, 0, false, n->as.prop_decl.body, n->line, n->col);
            break;
        }
        case AST_VAR_DECL: {
            for (size_t i = 0; i < n->as.var_decl.n_names; i++) {
                if (!top_level || !map_find(scope->symbols, n->as.var_decl.names[i])) {
                    register_name(s, scope, n->as.var_decl.names[i], SYM_VAR, n, n->line, n->col);
                }
            }
            for (size_t i = 0; i < n->as.var_decl.n_values; i++) {
                sema_expr(s, scope, n->as.var_decl.values[i]);
            }
            /* 推断类：var x = ClassName() */
            if (n->as.var_decl.n_values == 1 && n->as.var_decl.n_names >= 1) {
                const AstNode *v = n->as.var_decl.values[0];
                if (v && v->kind == AST_CALL && v->as.call.callee->kind == AST_IDENT) {
                    const Symbol *sym =
                        lookup_ident(s, scope, v->as.call.callee->as.ident.name);
                    if (sym && sym->kind == SYM_CLASS) {
                        Symbol *reg =
                            (Symbol *)map_find(scope->symbols, n->as.var_decl.names[0]);
                        if (reg) reg->inferred_class = sym->decl;
                    }
                }
            }
            break;
        }
        case AST_ASSIGN:
            for (size_t i = 0; i < n->as.assign.n; i++) {
                const AstNode *t = n->as.assign.targets[i];
                if (t->kind == AST_IDENT) {
                    const Symbol *sym = lookup_ident(s, scope, t->as.ident.name);
                    if (sym && sym != &s->builtin_sym && sym->kind == SYM_VAR && sym->decl) {
                        if (sym->decl->kind == AST_VAR_DECL && sym->decl->as.var_decl.is_const) {
                            sema_error_at(s, t->line, t->col, "cannot assign to constant '%s'",
                                          t->as.ident.name);
                        } else if (sym->decl->kind == AST_PROP_DECL) {
                            sema_error_at(s, t->line, t->col,
                                          "cannot assign to read-only property '%s'",
                                          t->as.ident.name);
                        }
                    }
                }
                sema_expr(s, scope, t);
            }
            for (size_t i = 0; i < n->as.assign.n_values; i++)
                sema_expr(s, scope, n->as.assign.values[i]);
            break;
        case AST_EXPR_STMT:
            if (!is_stmt_expr(n->as.expr_stmt.expr)) {
                sema_error_at(s, n->line, n->col,
                              "bare expression is not allowed as a statement");
            } else {
                sema_expr(s, scope, n->as.expr_stmt.expr);
            }
            break;
        case AST_IF:
            sema_if(s, scope, n, false); /* 语句上下文：分支不产值 */
            break;
        case AST_WHILE:
            sema_while(s, scope, n, false);
            break;
        case AST_FOR_IN:
            sema_for(s, scope, n, false);
            break;
        case AST_BREAK:
            if (s->loop_depth == 0)
                sema_error_at(s, n->line, n->col, "break outside a loop");
            break;
        case AST_CONTINUE:
            if (s->loop_depth == 0)
                sema_error_at(s, n->line, n->col, "continue outside a loop");
            break;
        case AST_RETURN:
            if (s->func_depth == 0)
                sema_error_at(s, n->line, n->col, "return outside a function");
            for (size_t i = 0; i < n->as.return_.n; i++)
                sema_expr(s, scope, n->as.return_.values[i]);
            break;
        case AST_THROW:
            sema_expr(s, scope, n->as.throw_.expr);
            break;
        case AST_MATCH:
            sema_expr(s, scope, n); /* match 语句复用表达式语义（臂值检查） */
            break;
        case AST_TRY_BLOCK: {
            /* try { } 作语句：与表达式位置（sema_expr AST_TRY_BLOCK）同样检查块内容。
               否则 try{} 子树整棵跳过 sema——未定义变量/break/return 上下文全不查（弱编译期检查）。 */
            Scope *bs = new_scope(s, scope);
            sema_block_contents(s, bs, n->as.try_block.body, true);
            break;
        }
        case AST_DO_CATCH: {
            /* do { } catch 作语句：镜像 sema_expr AST_DO_CATCH 的检查（块 + catch 处理器）。 */
            Scope *bs = new_scope(s, scope);
            sema_block_contents(s, bs, n->as.do_catch.body, true);
            const CatchHandler *h = &n->as.do_catch.handler;
            if (h->kind == CATCH_FALLBACK) {
                sema_expr(s, scope, h->fallback);
            } else if (h->kind == CATCH_BLOCK) {
                Scope *cs = new_scope(s, scope);
                if (h->error_name) {
                    Symbol sym = {SYM_VAR, NULL, NULL};
                    map_push_v(cs->symbols, h->error_name, &sym);
                }
                sema_block_contents(s, cs, h->fallback, true);
            } else { /* CATCH_MATCH */
                for (size_t i = 0; i < h->n_arms; i++) {
                    Scope *as = h->arms[i].n_payload > 0 ? new_scope(s, scope) : scope;
                    if (h->arms[i].n_payload > 0) {
                        for (size_t j = 0; j < h->arms[i].n_payload; j++) {
                            Symbol sym = {SYM_VAR, NULL, NULL};
                            map_push_v(as->symbols, h->arms[i].payload[j], &sym);
                        }
                    }
                    sema_expr(s, as, h->arms[i].value);
                }
            }
            break;
        }
        case AST_BLOCK: {
            Scope *bs = new_scope(s, scope);
            sema_block_contents(s, bs, n, false);
            break;
        }
        case AST_BLOCK_EXPR: {
            Scope *bs = new_scope(s, scope);
            sema_block_contents(s, bs, n, true);
            break;
        }
        case AST_DEFER: {
            Scope *ds = new_scope(s, scope);
            sema_block_contents(s, ds, n->as.defer_.block, false);
            break;
        }
        case AST_FUNC_DECL: {
            if (n->as.func.name &&
                (!top_level || !map_find(scope->symbols, n->as.func.name))) {
                register_name(s, scope, n->as.func.name, SYM_FUNC, n, n->line, n->col);
            }
            const bool saved_method = s->cur_in_method;
            const AstNode *saved_class = s->cur_class_node;
            s->cur_in_method = false; /* 具名函数是新函数，非方法 */
            s->cur_class_node = NULL;
            sema_func_body(s, scope, n);
            s->cur_in_method = saved_method;
            s->cur_class_node = saved_class;
            break;
        }
        case AST_CLASS_DECL: {
            if (!top_level || !map_find(scope->symbols, n->as.class_.name)) {
                register_name(s, scope, n->as.class_.name, SYM_CLASS, n, n->line, n->col);
            }
            if (n->as.class_.is_extern) break; /* extern 类主体是接口声明（extern var/func），不语义检查 */
            Scope *cs = new_scope(s, scope);
            for (size_t i = 0; i < n->as.class_.n_members; i++) {
                const AstNode *m = n->as.class_.members[i];
                if (m->kind != AST_FUNC_DECL || !m->as.func.name) continue;
                const bool dup = map_has(cs->symbols, m->as.func.name);
                /* 允许 getter + setter 同名（同一属性的一对） */
                bool prop_pair = false;
                if (dup && m->as.func.is_property) {
                    const Symbol *prev = (const Symbol *)map_find(cs->symbols, m->as.func.name);
                    if (prev && prev->decl && prev->decl->kind == AST_FUNC_DECL &&
                        prev->decl->as.func.is_property &&
                        prev->decl->as.func.is_setter != m->as.func.is_setter) {
                        prop_pair = true;
                    }
                }
                if (dup && !prop_pair) {
                    sema_error_at(s, m->line, m->col, "duplicate declaration of '%s'",
                                  m->as.func.name);
                } else if (!dup) {
                    Symbol sym = {SYM_FUNC, m, NULL};
                    map_push_v(cs->symbols, m->as.func.name, &sym);
                }
                const bool saved_method = s->cur_in_method;
                const AstNode *saved_class = s->cur_class_node;
                if (!m->as.func.is_static) s->cur_in_method = true;
                s->cur_class_node = n; /* self 类推断 */
                sema_func_body(s, cs, m);
                s->cur_in_method = saved_method;
                s->cur_class_node = saved_class;
            }
            break;
        }
        case AST_ERROR_SET_DECL:
            if (!top_level || !map_find(scope->symbols, n->as.error_set.name)) {
                register_name(s, scope, n->as.error_set.name, SYM_ERR_SET, n, n->line, n->col);
            }
            break;
        case AST_ENUM_DECL:
            if (!top_level || !map_find(scope->symbols, n->as.enum_.name)) {
                register_name(s, scope, n->as.enum_.name, SYM_ENUM, n, n->line, n->col);
            }
            break;
        case AST_IMPORT: {
            const char *first = first_segment(s, n->as.import.name);
            if (first && !map_find(scope->symbols, first)) {
                register_name(s, scope, first, SYM_MODULE, n, n->line, n->col);
            }
            /* 末段短名：import System.Math 后 Math.sqrt(...) 也可用 */
            const char *last = last_segment(s, n->as.import.name);
            if (last && (first == NULL || strcmp(last, first) != 0) &&
                !map_find(scope->symbols, last)) {
                register_name(s, scope, last, SYM_MODULE, n, n->line, n->col);
            }
            for (size_t i = 0; i < n->as.import.n_names; i++) { /* 选择性导入成员：本作用域可用 */
                if (!map_find(scope->symbols, n->as.import.names[i])) {
                    register_name(s, scope, n->as.import.names[i], SYM_MODULE, n, n->line, n->col);
                }
            }
            break;
        }
        case AST_EXPORT_MODULE: {
            const char *first = first_segment(s, n->as.export_module.name);
            if (first && !map_find(scope->symbols, first)) {
                register_name(s, scope, first, SYM_MODULE, n, n->line, n->col);
            }
            break;
        }
        case AST_EXPORT:
            sema_stmt(s, scope, n->as.export_.decl, top_level);
            break;
        default:
            break;
    }
}

/* ---------- 顶层预声明（先注册所有名字，允许前向引用；顶层重声明=重新绑定） ---------- */

static void predeclare_name(Sema *s, Scope *scope, const char *name, SymKind kind,
                            const AstNode *decl) {
    (void)s;
    Symbol *sym = (Symbol *)map_find(scope->symbols, name);
    if (sym) {
        sym->kind = kind;
        sym->decl = decl;
        sym->inferred_class = NULL;
    } else {
        Symbol ns = {kind, decl, NULL};
        map_push_v(scope->symbols, name, &ns);
    }
}

static void predeclare(Sema *s, Scope *scope, const AstNode *n) {
    if (!n) return;
    switch (n->kind) {
        case AST_VAR_DECL:
            for (size_t i = 0; i < n->as.var_decl.n_names; i++) {
                predeclare_name(s, scope, n->as.var_decl.names[i], SYM_VAR, n);
            }
            break;
        case AST_PROP_DECL:
            predeclare_name(s, scope, n->as.prop_decl.name, SYM_VAR, n);
            break;
        case AST_FUNC_DECL:
            if (n->as.func.name)
                predeclare_name(s, scope, n->as.func.name, SYM_FUNC, n);
            break;
        case AST_CLASS_DECL:
            predeclare_name(s, scope, n->as.class_.name, SYM_CLASS, n);
            break;
        case AST_ERROR_SET_DECL:
            predeclare_name(s, scope, n->as.error_set.name, SYM_ERR_SET, n);
            break;
        case AST_ENUM_DECL:
            predeclare_name(s, scope, n->as.enum_.name, SYM_ENUM, n);
            break;
        case AST_IMPORT: {
            const char *first = first_segment(s, n->as.import.name);
            if (first && !map_find(scope->symbols, first)) {
                predeclare_name(s, scope, first, SYM_MODULE, n);
            }
            const char *last = last_segment(s, n->as.import.name);
            if (last && (first == NULL || strcmp(last, first) != 0) &&
                !map_find(scope->symbols, last)) {
                predeclare_name(s, scope, last, SYM_MODULE, n);
            }
            for (size_t i = 0; i < n->as.import.n_names; i++) {
                if (!map_find(scope->symbols, n->as.import.names[i])) {
                    predeclare_name(s, scope, n->as.import.names[i], SYM_MODULE, n);
                }
            }
            break;
        }
        case AST_EXPORT_MODULE: {
            const char *first = first_segment(s, n->as.export_module.name);
            if (first && !map_find(scope->symbols, first)) {
                predeclare_name(s, scope, first, SYM_MODULE, n);
            }
            break;
        }
        case AST_EXPORT:
            predeclare(s, scope, n->as.export_.decl);
            break;
        default:
            break;
    }
}

/* ---------- 对外 API ---------- */

Sema *sema_new(void) {
    Sema *s = calloc(1, sizeof(Sema));
    assert(s != NULL);
    s->arena = arena_new();
    s->builtin_sym.kind = SYM_FUNC;
    s->builtin_sym.decl = NULL;
    s->builtin_sym.inferred_class = NULL;
    return s;
}

void sema_free(Sema *s) {
    if (!s) return;
    for (MapNode *node = s->maps; node;) {
        MapNode *next = node->next;
        map_free(node->map);
        free(node);
        node = next;
    }
    free(s->external);
    arena_free(s->arena);
    free(s);
}

void sema_add_external_global(Sema *s, const char *name) {
    for (size_t i = 0; i < s->n_external; i++) {
        if (strcmp(s->external[i], name) == 0) return;
    }
    if (s->n_external == s->cap_external) {
        size_t ncap = s->cap_external ? s->cap_external * 2 : 16;
        s->external = realloc(s->external, ncap * sizeof(const char *));
        assert(s->external != NULL);
        s->cap_external = ncap;
    }
    /* 拷贝进 arena：不保存调用方裸指针（mica.c 传 globals 迭代的临时指针，sema 生命周期
       内一旦有分配（arena_copy_str/注册）调用方缓冲可能失效）。 */
    s->external[s->n_external++] = arena_copy_str(s->arena, name, strlen(name));
}

size_t sema_error_count(const Sema *s) {
    return s->n_errors;
}

const ParserError *sema_error(const Sema *s, size_t i) {
    return &s->errors[i];
}

void sema_analyze(Sema *s, AstNode **stmts, size_t n) {
    s->global = new_scope(s, NULL);
    /* 外部全局（宿主注入或先前运行产生）注册为已知全局，避免报未定义 */
    for (size_t i = 0; i < s->n_external; i++) {
        Symbol sym = {SYM_VAR, NULL, NULL};
        map_push_v(s->global->symbols, s->external[i], &sym);
    }
    for (size_t i = 0; i < n; i++) {
        predeclare(s, s->global, stmts[i]);
    }
    for (size_t i = 0; i < n; i++) {
        sema_stmt(s, s->global, stmts[i], true);
        if (s->fatal) break;
    }
}
