#ifndef MICA_ARRAY_H
#define MICA_ARRAY_H

#include <stddef.h>

/* 定长元素动态数组（类似 std::vector）。元素按字节拷贝，适合 POD。 */
typedef struct Array Array;

Array *arr_new(size_t element_size);
void arr_free(Array *self);

/* 确保容量 >= capacity（仅扩容，保留已有元素与 size）。 */
void arr_reserve(Array *self, size_t capacity);

/* 在末尾追加：push_v 拷贝元素内容；push 返回新槽位由调用方写入。 */
void *arr_push_v(Array *self, const void *element);
void *arr_push(Array *self);
void *arr_pop(Array *self);   /* 空数组返回 NULL */

void *arr_begin(const Array *self);
void *arr_end(const Array *self);
void *arr_last(const Array *self);   /* 空数组断言失败 */
void *arr_at(const Array *self, size_t index);   /* 越界返回 NULL */

size_t arr_size(const Array *self);
size_t arr_capacity(const Array *self);
size_t arr_element_size(const Array *self);

#endif /* MICA_ARRAY_H */
