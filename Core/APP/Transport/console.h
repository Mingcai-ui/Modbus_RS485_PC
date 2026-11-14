#ifndef __CONSOLE_H__
#define __CONSOLE_H__

/*
 * console.h —— 日志通道（Transport 层）
 *
 * 【解决什么问题】
 *   重构前 printf 走 USART1，而 USART1 同时是 PC 的 Modbus 二进制链路。
 *   ASCII 日志字节插进二进制帧之间，PC 端会看到乱码、CRC 必然校验失败。
 *   本层把 printf 出口改到独立的 USART3，两条链路物理隔离。
 *
 * 【实现关键】
 *   printf → _write()，而 CubeMX 生成的 syscalls.c 里那个 _write 是弱符号
 *   （__attribute__((weak))），console.c 提供同名强符号即可自动覆盖，
 *   无需修改任何 CubeMX 生成文件。
 *
 * 【注意】
 *   - USART3 的波特率/引脚由 CubeMX 配置（MX_USART3_UART_Init），
 *     本层只管"往哪写"，不管"怎么配"；
 *   - 当前 DHT11_LOG_ENABLE = 0，平时无日志输出，不影响实时性；
 *   - 发送是阻塞的，不要在中断或临界区里 printf。
 */
void console_init(void);

#endif /* __CONSOLE_H__ */
