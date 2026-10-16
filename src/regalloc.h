#ifndef MICA_REGALLOC_H
#define MICA_REGALLOC_H

#include <stdint.h>

#include "proto.h"

/* 图着色寄存器分配（后处理，模块化独立于 codegen）：
   - 输入：codegen 产出的 proto + arity + 当前 max_regs（虚拟寄存器数）。
   - 算法：构建 CFG → 逆向数据流 liveness → 干扰图（live-in 两两 + def vs live-out）
     → 连续寄存器组（CALL 实参/结果、ARRAY/MAP 元素块）union-find 合并
     → 贪心图着色（固定寄存器预着色，块连续着色）。
   - 固定寄存器（不可重映射）：R0（slot0）、1..arity（参数）、被捕获局部
     （proto 中 OP_CLOSURE 引用的子函数 capval 描述 index）。
   - 成功：改写 proto 指令（寄存器字段重映射），返回新 max_regs（>0）；
     失败（如连续组无法着色）：proto 保持不变，返回 -1。 */
int regalloc_run(Proto *proto, uint32_t arity, uint32_t max_regs);

#endif /* MICA_REGALLOC_H */
