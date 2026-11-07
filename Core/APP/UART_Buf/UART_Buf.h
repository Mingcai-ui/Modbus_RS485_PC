#ifndef __UART_BUF_H__
#define __UART_BUF_H__

#include "modbus_rtu.h"

extern uint8_t uart1_RxData;          // 串口1接收的临时单字节
extern uint8_t uart2_RxData;          // 串口2接收的临时单字节

extern char PC_RX[BUFFERSIZE];         /* UART1中断写入的原始接收区 */
extern char ModbusRx[BUFFERSIZE];      /* UART2中断写入的原始接收区 */


extern volatile uint8_t DataCounter_PC;       /* 接收数据计数器 */
extern volatile uint8_t DataCounter_Modbus;       /* 接收数据计数器 */
#endif