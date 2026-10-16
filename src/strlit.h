#ifndef MICA_STRLIT_H
#define MICA_STRLIT_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "str.h"
#include "obj.h" /* obj_string_hash 访问器（NAME_EQ 用于 ObjString 懒哈希） */

/* ============================================================
   热路径字面量名字比较（VM 成员访问 / 内置方法分派 / 类型名匹配）。

   Mica 字符串是 ObjString（含 len 与创建时缓存的 32 位 MurmurHash3 哈希）。
   C 风格字符串比较（strcmp）在 VM 内层循环里太贵：先比较长度与哈希
   （两个寄存器比较）就能拒绝绝大多数不匹配，命中后再 memcmp 内容兜底
   （防御 32 位哈希碰撞）。

   NAME_EQ(s, h, lit)：s 是 ObjString*，lit 是字符串字面量，h 是
   fnv_hash(lit, sizeof(lit)-1) 的编译期常量。先比长度与哈希，
   再 memcmp。所有 H_* 常量由 tools/strhash.py 生成（MurmurHash3 x86_32，
   与 src/str.c fnv_hash 一致）；改 fnv_hash 或字面量内容须同步
   （test_vm 的 test_strlit_hashes 会校验）。

   注意：NAME_EQ 的参数名用 h（勿用 hash）——ObjString 字段也叫 hash，
   宏展开会把 (s)->hash 里的 hash 误替换成实参。
   ============================================================ */

/* NAME_EQ 的 s 是 ObjString*；用 obj_string_hash 访问器（懒哈希：未缓存则计算），
   勿直接读 (s)->hash（ObjString 懒哈希下未计算时是 0/垃圾）。 */
#define NAME_EQ(s, h, lit)                                                     \
    ((s)->len == (sizeof(lit) - 1) && obj_string_hash(s) == (h) &&              \
     memcmp((s)->chars, (lit), sizeof(lit) - 1) == 0)

/* 同上，但 s 是 const String*（标识符池的 SSO 字符串）：str_len/str_data 取内联内容。 */
#define STR_EQ(s, h, lit)                                                      \
    (str_len(s) == (sizeof(lit) - 1) && (s)->hash == (h) &&                     \
     memcmp(str_data(s), (lit), sizeof(lit) - 1) == 0)

/* ---------- 内置类型名 ---------- */
#define H_NUMBER             0x4E0101D7u
#define H_INT                0xCE831DDFu
#define H_FLOAT              0x95FB9FE8u
#define H_BOOL               0x8FD0D24Au
#define H_STRING             0xAD217AABu
#define H_ARRAY              0x13513BF2u
#define H_MAP                0xEB1D32C8u
#define H_NULL               0x66F07992u
#define H_ERROR              0x640990D2u
#define H_ENUM               0xB8BBEA07u
#define H_COROUTINE          0x5E820CB0u
#define H_FUNCTION           0x3D75DC7Au
#define H_OBJECT             0x055EC73Du

/* ---------- get_member 特殊属性名 ---------- */
#define H_LENGTH             0x0ECEF0C1u
#define H_MESSAGE            0x14820755u
#define H_PAYLOAD            0x42D04267u
#define H_RESUME             0xF5F7CD6Cu
#define H_CANCEL             0x6C656F2Cu
#define H_STATUS             0xCA52CCB5u
#define H_CREATE             0x2FE30BDFu
#define H_NAME               0xDBAF43F0u
#define H_FULL_NAME          0xA8989129u
#define H_BASE_TYPE          0x88D36C56u
#define H_ASSEMBLY           0x88542C16u
#define H_ARITY              0x4D896D0Bu
#define H_IS_STATIC          0xA9BE8D80u
#define H_MODULE             0xAE3684A4u
#define H_CAN_READ           0x30E9FCBAu
#define H_CAN_WRITE          0x9BE4AD76u
#define H_IS_PRIMITIVE       0xD19BF39Du
#define H_IS_CLASS           0x2947C448u
#define H_IS_NULL            0xF08FBDA6u
#define H_IS_BOOL            0x56BADC80u
#define H_IS_NUMBER          0x3299F1FDu
#define H_IS_STRING          0x50B7A96Bu
#define H_IS_ARRAY           0xC50BCA46u
#define H_IS_MAP             0xE68C281Fu
#define H_IS_FUNCTION        0x7D93BB60u
#define H_IS_ERROR           0x30012AEAu
#define H_IS_COROUTINE       0xDF7777A4u
#define H_IS_WEAKREF         0xF36E524Bu

/* ---------- 内置类型成员方法 ---------- */
#define H_PUSH               0x0F308B8Cu
#define H_POP                0x960361FFu
#define H_JOIN               0xEF86F0DAu
#define H_CLEAR              0x7857FAAEu
#define H_REVERSE            0xD8D820F4u
#define H_CONTAINS           0x9338EEC0u
#define H_REPLACE            0x54B94882u
#define H_TRIM               0x77EB7ABAu
#define H_STARTS_WITH        0xCF2686D2u
#define H_ENDS_WITH          0x685DCFB8u
#define H_TO_UPPER           0xDA580A42u
#define H_TO_LOWER           0xE117B37Au
#define H_SUBSTRING          0xAA394B42u
#define H_INDEX_OF           0x4A18877Au
#define H_SPLIT              0x916F9388u
#define H_REPEAT             0x26374320u
#define H_HAS                0xD547087Cu
#define H_GET                0x3B6D35A2u
#define H_REMOVE             0x3D2727DDu
#define H_SIZE               0xED8917FAu
#define H_KEYS               0x97E36BE1u
#define H_VALUES             0x33B551C8u
#define H_IS_VALID           0x2998BA61u
#define H_METHODS            0xF0D8F3AEu
#define H_STATIC_METHODS     0xA9CA92D7u
#define H_FIELDS             0x4DE26E52u
#define H_PROPERTIES         0x61716144u
#define H_GET_CLASS          0xB7D2D5D9u
#define H_SET                0x5ECC6FE8u
#define H_INVOKE             0xAA457029u
#define H_TYPES              0x871B2836u
#define H_GET_TYPE           0x10550649u
#define H_FUNCTIONS          0x17DDB49Bu
#define H_MODULES            0xC6CBB55Eu
#define H_GET_MODULE         0x441F262Cu



#endif /* MICA_STRLIT_H */
