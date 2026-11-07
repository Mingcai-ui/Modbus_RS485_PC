#ifndef __MODBUS_RTU_H__
#define __MODBUS_RTU_H__

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* ================== 宏定义 ================== */
#define SLAVEID             1       /* 从机地址 */
#define NUMBER_OF_REGISTER  10      /* 保持寄存器总数 (0x03/0x06/0x10) */
#define NUMBER_OF_COIL      30      /* 线圈总数     (0x01/0x05/0x0F) */

/* 缓冲区取 uint8_t 计数器上限 255，可容纳 Modbus 最大帧 (256 字节级) */
#define BUFFERSIZE          255

/* 帧间静默判帧时间 (ms)
 * 9600bps 8N1 : 1 字符 = 10bit / 9600 = 1.04ms , 3.5T ≈ 3.65ms
 * 取 5ms 留出 RS485 收发器切换余量；SysTick 1ms 计数，精度 1ms 够用 */
#define FRAME_GAP_MS        5

/* Modbus 异常码 */
#define EX_ILLEGAL_FUNCTION 0x01    /* 从机不支持该功能码 */
#define EX_ILLEGAL_ADDRESS  0x02    /* 数据地址越界 */
#define EX_ILLEGAL_VALUE    0x03    /* 数据值非法 */

/* ================== 全局数据区 (定义在 modbus_rtu.c) ================== */
extern uint16_t ModbusRegister[NUMBER_OF_REGISTER];
extern bool     ModbusCoil[NUMBER_OF_COIL];

/* ================== 功能码枚举 ================== */
enum
{
    ReadCoil              = 0x01,
    ReadHoldingRegister   = 0x03,
    WriteSingleCoil       = 0x05,
    WriteSingleRegister   = 0x06,
    WriteMultipleCoils    = 0x0F,
    WriteMultipleRegisters = 0x10,
};

/* ================== 接收状态 (中断与主循环共享，必须 volatile) ================== */
extern volatile uint8_t RxInterruptFlag;            //UART2数据接收状态标志位
extern volatile uint8_t uartTimeCounter;            //UART2数据间隔计数器
extern volatile uint8_t uartPacketComplatedFlag;    //UART2收到完整帧数据标志位

extern volatile uint8_t RxInterruptFlag_PC;         //UART1数据接收状态标志位
extern volatile uint8_t uartTimeCounter_PC;         //UART1数据间隔计数器
extern volatile uint8_t uartPacketComplatedFlag_PC; //UART1收到完整帧数据标志位 
/* ================== 接收与处理 ================== */
void uartDataHandler(void);                          /* main 的 while(1) 里轮询调用 */

void uartTimer(void);                                /* SysTick 中断 (1ms) 里调用 */
void uartTimer_PC(void);

void uartDataHandler_PC(void);
void uartDataHandler_Modbus(void);
void transmitDataMake(char *msg, uint8_t Lenght);

/* ================== 具体功能码 ================== */
void makePacket_01(char *msg, uint8_t Lenght);
void makePacket_03(char *msg, uint8_t Lenght);
void makePacket_05(char *msg, uint8_t Lenght);
void makePacket_06(char *msg, uint8_t Lenght);
void makePacket_15(char *msg, uint8_t Lenght);
void makePacket_16(char *msg, uint8_t Lenght);

/* ================== 工具函数 ================== */
void     sendMessage(char *msg, uint8_t len);
void     sendException(char *msg, uint8_t code);
uint8_t  findByte(int16_t NumberOfCoil);
void printf_HXL(const char *msg, uint8_t Lenght);

#endif /* __MODBUS_RTU_H__ */
