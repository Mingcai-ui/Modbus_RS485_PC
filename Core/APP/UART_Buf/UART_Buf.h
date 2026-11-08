#ifndef __UART_BUF_H__
#define __UART_BUF_H__

#include "modbus_rtu.h"

extern uint8_t uart1_RxData;          // 串口1接收的临时单字节
extern uint8_t uart2_RxData;          // 串口2接收的临时单字节

extern char PC_RX[BUFFERSIZE];         /* UART1中断写入的原始接收区 */
extern char ModbusRx[BUFFERSIZE];      /* UART2中断写入的原始接收区 */


extern volatile uint8_t DataCounter_PC;       /* 接收数据计数器 */
extern volatile uint8_t DataCounter_Modbus;       /* 接收数据计数器 */

/* USART1 本帧超出缓冲区：中断写满 BUFFERSIZE 后停写，尾部字节被静默丢弃。
 * 主循环只看得到"长度 = 255"，与一条合法的 255 字节长帧无法区分，
 * 所以这个事实只能由中断置位、由 uartDataHandler_PC 消费后清零。 */
extern volatile uint8_t frameOverflow_PC;
#endif