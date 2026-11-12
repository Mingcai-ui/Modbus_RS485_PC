#ifndef __MB_PORT_H__
#define __MB_PORT_H__

#include <stdint.h>
#include <stdbool.h>

/* 端口 = 物理通道的抽象编号。
 * 链路层只认这个编号，不认识 Modbus 地址、不认识功能码。 */
typedef enum
{
    MB_PORT_PC = 0,      /* USART1：PC / 上位机 / 调试日志 */
    MB_PORT_RS485,       /* USART2：RS485 总线（带 DE 方向控制） */
    MB_PORT_COUNT
} mb_port_t;

void mb_port_init(void);

/* 返回值：0 = 已发出，-1 = 参数非法 / 端口不存在。
 * 注意它是"阻塞发送"——函数返回时最后一个字节已移出（TC 置位），
 * 所以 RS485 的 DE 可以在返回后立刻拉低。 */
int mb_port_send(mb_port_t port, const uint8_t *buf, uint8_t len);

#endif /* __MB_PORT_H__ */
