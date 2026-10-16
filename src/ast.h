#ifndef MICA_AST_H
#define MICA_AST_H

#include <stdbool.h>
#include <stddef.h>

#include "token.h" /* TokenKind，用于 unary/binary/assign 的运算符字段 */

/* ---------- Arena（块式 bump 分配器） ---------- */

typedef struct Arena Arena;

Arena *arena_new(void);
void arena_free(Arena *a);
/* 分配 size 字节（对齐到 16），返回零初始化的内存。 */
void *arena_alloc(Arena *a, size_t size);
/* 拷贝 len 字节并以 NUL 结尾，返回 arena 中的指针。 */
char *arena_copy_str(Arena *a, const char *s, size_t len);

/* ---------- AST ---------- */

typedef struct AstNode AstNode;

typedef enum {
    /* 语句 / 声明 */
    AST_VAR_DECL,         /* var/let 声明 */
    AST_PROP_DECL,        /* let name => body：只读计算属性（读取时求值，禁 $N/yield） */
    AST_ASSIGN,           /* 赋值（含复合赋值与多重赋值） */
    AST_EXPR_STMT,        /* 表达式语句 */
    AST_IF,               /* if / elif / else */
    AST_WHILE,
    AST_FOR_IN,           /* for 变量 in 集合 [step] */
    AST_BREAK,
    AST_CONTINUE,
    AST_RETURN,           /* return [值列表] */
    AST_BLOCK,            /* 语句块 */
    AST_FUNC_DECL,        /* 具名/匿名函数声明 */
    AST_CLASS_DECL,       /* class */
    AST_ERROR_SET_DECL,   /* error Name { Case(...) } */
    AST_ENUM_DECL,        /* enum Name { A, B = 1, C }：成员为自动递增常量 */
    AST_IMPORT,           /* import 模块名 */
    AST_EXPORT_MODULE,    /* export module 模块名 */
    AST_EXPORT,           /* export <声明> */

    /* 表达式 */
    AST_NUMBER,
    AST_STRING,           /* 分段：字面量 + 插值 */
    AST_NULL,
    AST_TRUE,
    AST_FALSE,
    AST_UNINITIALIZED,
    AST_IDENT,
    AST_ARRAY,
    AST_MAP,
    AST_ERROR_VAL,        /* error("msg") */
    AST_UNARY,
    AST_BINARY,
    AST_TERNARY,
    AST_CALL,             /* 调用：位置 + 命名参数 */
    AST_INDEX,
    AST_MEMBER,
    AST_LAMBDA,           /* (参数) => 表达式 | { 块 } */
    AST_VARARGS,          /* ...（varargs 值 / 展开） */
    AST_BLOCK_EXPR,       /* get { ... }：块表达式（末句表达式作值；调用后加 ; 则不返回其值） */
    AST_TRY,              /* try / try? / try! 表达式 */
    AST_TRY_BLOCK,        /* try { stmts } / try! { stmts }：语句块，块内错误上抛/终止 */
    AST_CATCH,            /* expr catch <处理器> */
    AST_DO_CATCH,         /* do { stmts } catch <处理器>：语句（不可作表达式），块内错误进 catch */
    AST_THROW,            /* throw expr：抛出 error 值（函数内返回 error；顶层终止程序） */
    AST_MATCH,            /* match expr { 模式 => 值, ... }：模式匹配表达式/语句 */
    AST_THIS_FUNC,        /* this_func：当前函数自身 */
    AST_YIELD,            /* yield [expr] */
    AST_DEFER,            /* defer { block } */
    AST_OPTIONAL,         /* a?.b / a?.b(...)：可选链（a 为 null 得 null，否则成员访问/调用） */

    AST_KIND_COUNT
} AstKind;

typedef struct {
    const char *name;
    bool is_self;          /* 接收者参数（默认 self，或 别名: self） */
    bool has_default;
    AstNode *default_expr;
} AstParam;

typedef struct {
    bool named;
    const char *name;      /* named 时的参数名 */
    AstNode *expr;
} AstArg;

typedef struct {
    AstNode *key;
    AstNode *value;
} AstMapEntry;

typedef struct {
    bool is_literal;
    const char *text;      /* 字面量：已解码文本（arena 拷贝） */
    size_t len;
    AstNode *expr;         /* 插值表达式 */
} AstStrPart;

/* 匹配臂模式（catch 与 match 共用；catch 只用 ARM_ENUM，match 支持全部）。
   ARM_LITERAL：字面量相等（number/string/bool/null）；
   ARM_BIND：无条件匹配并绑定到标识符；
   ARM_WILDCARD：无条件匹配不绑定（_）；
   ARM_ENUM：枚举 case / 错误 case（set.case(payload) 或裸 case）。 */
typedef enum { ARM_LITERAL, ARM_BIND, ARM_WILDCARD, ARM_ENUM } ArmPatternKind;

typedef struct {
    ArmPatternKind pattern;
    AstNode *literal;      /* ARM_LITERAL：字面量节点 */
    const char *bind;      /* ARM_BIND：绑定名 */
    const char *set_name;  /* ARM_ENUM：set 名（NULL 表示裸 case） */
    const char *case_name;
    const char **payload;  /* ARM_ENUM：绑定的载荷标识符 */
    size_t n_payload;
    AstNode *value;
} AstMatchArm;

typedef enum { CATCH_FALLBACK, CATCH_BLOCK, CATCH_MATCH } CatchKind;

typedef struct {
    CatchKind kind;
    const char *error_name;   /* CATCH_BLOCK：错误绑定标识符 */
    AstNode *fallback;        /* CATCH_FALLBACK：兜底表达式；CATCH_BLOCK：body 块 */
    AstMatchArm *arms;        /* CATCH_MATCH */
    size_t n_arms;
} CatchHandler;

typedef struct {
    const char *name;
    const char **payload;
    size_t n_payload;
} AstErrorCase;

/* 枚举成员：payload 为成员载荷参数名（NULL/0 = 无 payload，成员为整数常量）；
   value 为显式值表达式（NULL = 自动递增到上一个成员 + 1）。 */
typedef struct {
    const char *name;
    const char **payload;
    size_t n_payload;
    AstNode *value;
} AstEnumMember;

struct AstNode {
    AstKind kind;
    int line;
    int col;
    union {
        struct { const char *text; size_t len; } number;
        struct { AstStrPart *parts; size_t n_parts; } str;
        struct { const char *name; } ident;
        struct { AstNode **elems; size_t n; } array;
        struct { AstMapEntry *entries; size_t n; } map;
        struct { AstNode *msg; } error_val;
        struct { int op; AstNode *operand; } unary;
        struct { int op; AstNode *left; AstNode *right; } binary;
        struct { AstNode *cond; AstNode *then_; AstNode *else_; } ternary;
        struct { AstNode *callee; AstArg *args; size_t n_args; } call;
        struct { AstNode *obj; AstNode *index; } index;
        struct { AstNode *obj; const char *name; } member;
        struct { AstParam *params; size_t n_params; bool has_varargs; bool is_throws; AstNode *body; } lambda;
        struct { int kind; AstNode *expr; } try_;            /* kind=TOK_TRY/TRY_QUESTION/TRY_BANG */
        struct { int kind; AstNode *body; } try_block;       /* kind=TOK_TRY/TOK_TRY_BANG；body=块 */
        struct { AstNode *expr; CatchHandler handler; } catch_;
        struct { AstNode *body; CatchHandler handler; } do_catch; /* do { body } catch handler（语句） */
        struct { AstNode *expr; } throw_;                    /* throw expr */
        struct { AstNode *expr; AstMatchArm *arms; size_t n_arms; } match_; /* match expr { 臂, 臂 } */
        struct { AstNode *expr; } yield_;                    /* yield [expr] */
        struct { AstNode *block; } defer_;                   /* defer { block } */
        struct { AstNode *obj; const char *name; AstArg *args; size_t n_args; bool is_call; } optional; /* a?.b / a?.b(...) */

        struct { const char **names; size_t n_names; AstNode **values; size_t n_values; bool is_const; } var_decl;
        struct { const char *name; AstNode *body; bool is_static; } prop_decl;
        struct { AstNode **targets; AstNode **values; size_t n; size_t n_values; int op; } assign;
        struct { AstNode *expr; bool no_value; } expr_stmt; /* no_value：; 结尾的调用语句不产值 */
        struct { AstNode *cond; AstNode *then_; AstNode **elif_conds; AstNode **elif_bodies; size_t n_elif; AstNode *else_; } if_;
        struct { AstNode *cond; AstNode *body; } while_;
        struct { const char **vars; size_t n_vars; AstNode *iterable; AstNode *step; AstNode *body; } for_in;
        struct { AstNode **values; size_t n; } return_;
        struct { AstNode **stmts; size_t n; } block;
        struct { const char *name; AstParam *params; size_t n_params; bool has_varargs; bool is_static; bool is_throws; bool is_property; bool is_setter; AstNode *body; } func;
        struct { const char *name; const char *parent; AstNode **members; size_t n_members; bool is_extern; } class_;
        struct { const char *name; AstErrorCase *cases; size_t n_cases; } error_set;
        struct { const char *name; AstEnumMember *members; size_t n_members; } enum_;
        struct { const char *name; const char **names; size_t n_names; } import; /* names: 选择性导入成员（空=整模块） */
        struct { const char *name; } export_module;
        struct { AstNode *decl; } export_;
    } as;
};

/* 在 arena 中分配一个新节点（kind + 位置），返回零初始化节点。 */
AstNode *ast_new(Arena *arena, AstKind kind, int line, int col);

/* 打印一棵表达式（内联）。 */
void ast_print_expr(const AstNode *node);

/* 打印一个语句/声明（带缩进）。 */
void ast_print_stmt(const AstNode *node, int indent);

/* 打印整个程序（顶层语句序列）。 */
void ast_print_program(const AstNode *const *stmts, size_t n);

#endif /* MICA_AST_H */
