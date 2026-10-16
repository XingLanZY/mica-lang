#ifndef MICA_BUILTIN_H
#define MICA_BUILTIN_H

#include "obj.h"

/* 内置类型成员方法与标准库底层 native（System.Builtin）。
   实现集中在 builtin.c；get_member（vm）经 find_builtin_method 查表分派。
   独立编译单元：只依赖 mica.h（不透明类型）+ vm.h + obj/value/gc/table，
   不触碰 mica_context 内部结构（结构定义私有于 mica.c）。

   扩展点：给数组/字符串/Map 添加成员方法 = 在 builtin.c 实现一个
   `static uint32_t bm_xxx(void*, void*, Value*, uint32_t, Value*)`，
   再在对应方法表 k_array_methods / k_string_methods / k_map_methods 加一行。
   VM、GC、codegen 均零改动。 */

typedef struct Gc Gc;
typedef struct ObjTable ObjTable;

/* 按内置类型查成员方法（array/string/map/weakref/反射对象），未命中返回 NULL。
   name 是 ObjString*：先比哈希+长度（strlit.h 常量）再 memcmp 内容，不做 strcmp。
   *_kv 变体接受原始键（hash/chars/len），供标识符池（SSO String）直接查找。 */
mica_native_fn find_builtin_method(uint32_t type, ObjString *name);
mica_native_fn find_builtin_method_kv(uint32_t type, uint32_t hash, const char *chars, size_t len);

/* 安装全局内建（print/typeof）与 System.Builtin 命名空间（编译器开洞）。
   在 mica_runtime 创建时调用一次。 */
void install_natives(Gc *gc, ObjTable *globals);
void install_builtin_module(Gc *gc, ObjTable *globals);

/* 创建 WeakRef 类对象（permanent；native_ctor 让 `WeakRef(obj)` 返回 ObjWeakRef）。
   供 extern class WeakRef（System.Runtime）注入用。 */
ObjClass *mica_weakref_class(Gc *gc);

#endif /* MICA_BUILTIN_H */
