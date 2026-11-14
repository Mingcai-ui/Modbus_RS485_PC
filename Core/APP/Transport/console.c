#include "console.h"
#include "usart.h"

void console_init(void)
{
    /* USART3 已由 CubeMX 的 MX_USART3_UART_Init() 初始化（含波特率/引脚），
     * 本层只负责"往哪写"，不负责"怎么配"，所以这里暂无额外配置。
     * 保留此函数是为了给以后加日志缓冲、加互斥留出接入点。 */
}

/* printf 重定向到 USART3（阻塞发送）
 *
 * 【为什么这个函数能生效，而不用改 syscalls.c】
 *   编译器把 printf 最终翻译成对 _write() 的调用。CubeMX 生成的 syscalls.c
 *   里已经有一个 _write()，但它带 __attribute__((weak))，是"弱符号"——
 *   当链接器发现别处还有同名强符号时，会自动丢弃弱符号、采用强符号。
 *   本文件的 _write() 就是强符号，于是链接后 printf 自然走到这里。
 *   好处：完全不用改动 CubeMX 生成文件，以后重新生成代码不会冲突被覆盖。
 *
 * 【为什么切到 USART3】
 *   原来 _write 走 USART1，而 USART1 同时是 PC 的 Modbus 二进制链路。
 *   ASCII 日志字节插进二进制帧流，PC 端会看到乱码、CRC 必然校验失败。
 *   把日志赶到独立的 USART3，两条链路物理隔离，互不干扰。
 *
 * 【为什么用阻塞发送】
 *   日志本身是尽力的调试手段，不需要中断/DMA 的复杂度；
 *   阻塞发送保证返回时数据已进移位寄存器，不会与后续操作抢缓冲。
 *   timeout 给 0xffff 是因为 115200 下打印一行几十字节远小于此值。 */
int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, 0xffff);
    return len;
}
