/* 定长元素动态数组（类似 std::vector）。元素按字节拷贝，适合 POD。
   非 GC 托管：arr_free 释放。与 array.h 配套。 */

#include "array.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

typedef struct Array {
    size_t element_size;
    size_t size;
    size_t capacity;
    void *data;
} Array;

Array *arr_new(size_t element_size) {
    Array *self = calloc(1, sizeof(Array));
    if (!self) return NULL;
    self->element_size = element_size;
    return self;
}

void arr_free(Array *self) {
    if (!self) return;
    free(self->data);
    free(self);
}

/* 确保容量 >= capacity（仅扩容，保留已有元素与 size）。 */
void arr_reserve(Array *self, size_t capacity) {
    if (capacity <= self->capacity) return;
    size_t ncap = self->capacity ? self->capacity : 8;
    while (ncap < capacity) ncap *= 2;
    self->data = realloc(self->data, ncap * self->element_size);
    self->capacity = ncap;
}

/* 在末尾追加：push_v 拷贝元素内容；push 返回新槽位由调用方写入。 */
void *arr_push_v(Array *self, const void *element) {
    arr_reserve(self, self->size + 1);
    void *dst = (char *)self->data + self->size * self->element_size;
    memcpy(dst, element, self->element_size);
    self->size++;
    return dst;
}

void *arr_push(Array *self) {
    arr_reserve(self, self->size + 1);
    void *dst = (char *)self->data + self->size * self->element_size;
    self->size++;
    return dst;
}

void *arr_pop(Array *self) {   /* 空数组返回 NULL */
    if (self->size == 0) return NULL;
    self->size--;
    return (char *)self->data + self->size * self->element_size;
}

void *arr_begin(const Array *self) {
    return self->data;
}

void *arr_end(const Array *self) {
    return (char *)self->data + self->size * self->element_size;
}

void *arr_last(const Array *self) {   /* 空数组断言失败 */
    assert(self->size > 0);
    return (char *)self->data + (self->size - 1) * self->element_size;
}

void *arr_at(const Array *self, size_t index) {   /* 越界返回 NULL */
    if (index >= self->size) return NULL;
    return (char *)self->data + index * self->element_size;
}

size_t arr_size(const Array *self) {
    return self->size;
}

size_t arr_capacity(const Array *self) {
    return self->capacity;
}

size_t arr_element_size(const Array *self) {
    return self->element_size;
}
