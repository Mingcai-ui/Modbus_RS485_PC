#ifndef __MODBUS_RTU_H__
#define __MODBUS_RTU_H__

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* ================== 宏定义 ================== */
#define SLAVEID             1       /* 从机地址 */
#define NUMBER_OF_REGISTER  10      /* 保持寄存器总数 (0x03/0x06/0x10) */
#define NUMBER_OF_COIL      30      /* 线圈总数     (0x01/0x05/0x0F) */

/* 缓冲区长度 = Modbus 单帧合法上限 255 字节
 * （0x10 写满 123 个寄存器：1+1+2+2+1+246+2 = 255）。
 * 计数器是 uint8_t，255 刚好卡在上限，**不可改成 256** ——
 * 否则 DataCounter 会回绕成 0，整帧被当空帧丢弃。 */
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

typedef enum{
    FRAME_OK = 0,
    FRAME_Too_SHORT,    // 帧太短
    FRAME_Too_LONG,     // 帧太长（由中断的 overflow 标记折算而来）
    FRAME_CRC_ERR,      //CRC错误
}FrameCheckResult;

/* 丢弃计数：四类原因含义不同，分开记才有诊断价值。
 * 只在主循环上下文读写、不进中断、不跨线程，所以不需要 volatile。
 * 不加 static，是为了以后 OLED / 调试页能直接读这几个数。 */
extern uint32_t bad_SHORT;    /* 帧长 < 4：判帧阈值或链路问题 */
extern uint32_t bad_LONG;     /* 超出 BUFFERSIZE，尾部字节已被丢弃 */
extern uint32_t bad_CRC;      /* CRC 错：帧内容坏了 */
extern uint32_t bad_Addrs;    /* 地址既非 0x01（RS485 变送器）也非 0x02（本机 DHT11） */

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
