#ifndef MICA_ASSEMBLY_H
#define MICA_ASSEMBLY_H

#include <stddef.h>

#include "gc.h"
#include "obj.h"

/* 程序集：把整个程序（入口 + 所有 import 模块）的编译产物序列化为单个二进制文件。
   格式见 assembly.c 顶部。反序列化重建的对象全部为 permanent（GC 不回收）。 */

/* 序列化：module_names/module_inits 长度 n_modules（模块按依赖序，先加载的在前）；
   module_exports[i]/export_counts[i] 是第 i 个模块的导出名列表；entry 为入口函数。
   0=成功，1=失败。 */
int assembly_write(Gc *gc, const char *path, const char *const *module_names, size_t n_modules,
                   const char *const *const *module_exports, const size_t *export_counts,
                   ObjClosure *const *module_inits, ObjClosure *entry);

/* 反序列化：重建模块 init 函数数组、导出名列表与入口。
   *out_names / *out_inits / *out_exports（含其内部数组）由调用方 malloc，须释放。
   0=成功。 */
int assembly_read(Gc *gc, const char *path, char ***out_names, size_t *out_n,
                  char ****out_exports, size_t **out_export_counts,
                  ObjClosure ***out_inits, ObjClosure **out_entry);

#endif /* MICA_ASSEMBLY_H */
