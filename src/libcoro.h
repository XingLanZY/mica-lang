#ifndef MICA_LIBCORO_H
#define MICA_LIBCORO_H

/* libcoro：协程标准库模块（coroutine 命名空间 + 协程 C API）。
   独立模块（仿 librefl："lib*.{h,c} = 一个标准库模块"），不用 builtin。
   - 语言层 API：coroutine.create/resume/cancel/status（mica_register_module + native）。
   - C API：mica_yield/mica_callk（宿主 native 挂起协程，无 longjmp）+
     mica_coroutine_create/resume/status/cancel（宿主驱动协程）。 */

#include "mica.h"

/* mica_runtime_new 时注册 coroutine 模块（mica_register_module + 4 个 native）。 */
void libcoro_init(mica_runtime *rt);

#endif /* MICA_LIBCORO_H */
