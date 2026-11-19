#ifndef __DIAG_H__
#define __DIAG_H__

#include <stdint.h>

/*
 * diag.h —— 诊断计数器（应用层）
 *
 * 【解决什么问题】
 *   重构前 4 个 bad_* 全局变量直接定义在 modbus_rtu.c 里，被 main.c 的
 *   显示代码以 extern 方式直接读取，协议层与显示层因此耦合。
 *   而且以后每加一类丢弃原因就要加一个全局变量 + 一行 extern。
 *
 *   收进本层后：
 *     - 协议层只调 diag_inc(DIAG_xxx)，不知道谁在看；
 *     - 显示层只调 diag_get(DIAG_xxx)，不知道谁在加；
 *     - 新原因只需在枚举里加一项 + 名字表加一行。
 *
 * 【为什么不需要 volatile / 临界区】
 *   这些计数只在主循环上下文读写（协议处理和显示刷新都在 while(1) 里），
 *   中断不碰它们。一旦将来在中断里加计数，就必须改成 volatile。
 */

typedef enum
{
    DIAG_FRAME_SHORT = 0,   /* 帧长 < 4：判帧阈值或链路问题 */
    DIAG_FRAME_LONG,        /* 超出缓冲，尾部字节已丢弃 */
    DIAG_CRC_ERR,           /* CRC 错：帧内容坏了 */
    DIAG_BAD_ADDR,          /* 地址既非 0x01（变送器）也非 0x02（本机） */
    DIAG_COUNT              /* 计数器总数，兼作数组长度 */
} diag_id_t;

/* 计数 +1。非法 id 静默忽略（不越界写）。 */
void diag_inc(diag_id_t id);

/* 取当前值。非法 id 返回 0。 */
uint32_t diag_get(diag_id_t id);

/* 取名字（"SHORT"/"LONG"/"CRC"/"ADDR"），给显示层用，避免两边硬编码同一批字符串。 */
// const char *diag_name(diag_id_t id);

/* 全部清零 */
void diag_reset(void);

#endif /* __DIAG_H__ */
