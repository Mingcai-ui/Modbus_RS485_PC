/*
 * Modbus RTU Slave —— STM32F103C8T6 / RS485 移植版
 *
 * 原始来源 : Modbus_Lib.c  (Ümit Can Güveren, 2022)
 * 移植要点 :
 *   1. 串口从 USART2 改为 RS485 半双工，PB2 控制收发方向 (DE/RE)
 *   2. 发送由 IT 改为阻塞，保证 DE 拉低时最后一个字节已移位完成
 *   3. 判帧阈值由 100ms 改为 5ms (9600bps 下 3.5T ≈ 3.65ms)
 *   4. 补长度下限检查、地址越界检查、异常响应
 */
#include "modbus_rtu.h"
#include "main.h"
#include "stdio.h"
#include "CRC16.h"
#include "frame.h"
#include "usart.h"
#include "Mb_port.h"
/* ================== 全局数据区 ================== */
uint16_t ModbusRegister[2];
bool     ModbusCoil[NUMBER_OF_COIL];

char tempModbusRx[BUFFERSIZE];   /* 拷贝出来处理，避免处理期间被中断改写 */
char ModbusTx[BUFFERSIZE];       /* 应答组包区 */

static char pcFrame[BUFFERSIZE];   /* 拷出来再发，避免发送期间被中断改写 */
static char rsFrame[BUFFERSIZE];

uint32_t bad_SHORT = 0;     //帧太短次数记录变量
uint32_t bad_LONG = 0;      //帧太长次数记录变量
uint32_t bad_CRC = 0;       //CRC错误次数记录变量
uint32_t bad_Addrs = 0;     //地址不匹配次数记录变量

uint16_t rxCRC;

// extern UART_HandleTypeDef huart2;

/* ========================================================
 * 2. 功能码分发
 * ======================================================== */
void transmitDataMake(char *msg, uint8_t Lenght,const uint16_t *regs, uint16_t regCount)
{
    /* 最小帧长：Modbus 所有标准请求都是 8 字节起
     * （地址 + 功能码 + 2 字节起始地址 + 2 字节数量/数据 + 2 字节 CRC）。
     * 各 makePacket_* 内部要读 msg[2]~msg[5]，长度不够会读到上一条帧的残留
     * 字节，组出一个地址/数量全是垃圾的应答。这里统一挡掉，
     * 顺带覆盖 PC 侧 0x02 和 Modbus 侧本机从机两条入口。
     * 此时 msg[0]/msg[1] 必然有效（checkFrame 已保证至少 4 字节）。 */
    if (Lenght < 8)
    {
        sendException(msg, EX_ILLEGAL_VALUE);
        return;
    }

    switch (msg[1])
    {
    // case ReadCoil:               makePacket_01(msg, Lenght, regs, regCount); break;
    case ReadHoldingRegister:    makePacket_03(msg, Lenght, regs, regCount); break;
    // case WriteSingleRegister:    makePacket_06(msg, Lenght, regs, regCount); break;
    // case WriteSingleCoil:        makePacket_05(msg, Lenght, regs, regCount); break;
    // case WriteMultipleCoils:     makePacket_15(msg, Lenght, regs, regCount); break;
    // case WriteMultipleRegisters: makePacket_16(msg, Lenght, regs, regCount); break;

    default:
        sendException(msg, EX_ILLEGAL_FUNCTION);   /* 不支持的功能码 → 0x81 */
        break;
    }
}


/* 从机应答方向的出口规则：
 *   0x01 → RS485 上的外部变送器
 *   其他 → 本机从机（0x02）的应答，回 PC
 *
 * 注意：这条规则只在"应答方向"成立。主站发送时，发给 01/02 的请求
 * 都走同一条 485，地址不参与出口决策 —— 主站自己指定 port。
 * 第 4 步会把本函数搬到 mb_router.c。 */
static mb_port_t reply_port(uint8_t first_byte)
{
    return (first_byte == 0x01) ? MB_PORT_RS485 : MB_PORT_PC;
}

void sendMessage(char *msg, uint8_t len)
{
    (void)mb_port_send(reply_port((uint8_t)msg[0]), (const uint8_t *)msg, len);
}

//判断帧是否合法,进行长度检验和CRC检验
static FrameCheckResult checkFrame(const char *msg, uint8_t len)    //const防止修改
{
    if (len < 4)    return FRAME_Too_SHORT;/* 最短合法帧 = 从机地址 + 功能码 + CRC(2) = 4 字节 */

    return (MODBUS_CRC16(msg, len)?FRAME_CRC_ERR:FRAME_OK);
}

// PC 侧数据处理
void uartDataHandler_PC(void)
{
    uint8_t overflowed = 0;
    uint8_t len = frame_poll(FRAME_CH_PC, (uint8_t *)pcFrame, &overflowed);
    if (len == 0) return;

    /* 把"超长"折进同一个判决变量，下面只需要一处分支。
       超长的证据只存在于中断里，由 frame_poll 转出。 */
    FrameCheckResult verdict = overflowed ? FRAME_Too_LONG : checkFrame(pcFrame, len);

    if(verdict != FRAME_OK) //判断帧是否合法
    {
        if      (verdict == FRAME_Too_SHORT) { bad_SHORT++; }
        else if (verdict == FRAME_Too_LONG)  { bad_LONG++;  }
        else if (verdict == FRAME_CRC_ERR)   { bad_CRC++;   }
        return;
    }

    switch((uint8_t)pcFrame[0])     //路由层
    {
        /* 0x01：RS485 总线上的外部温湿度变送器。原样透传，由它自己应答；
           它的应答经 USART2 回来后由 uartDataHandler_Modbus 回显给 PC。 */
        case 0x01: sendMessage(&pcFrame[0],len);    break;

        case 0x02:
            if ((uint8_t)pcFrame[1] != ReadHoldingRegister)     //目前测试03功能码,该判断删除无影响,transmitDataMake会处理异常功能码
            {
                sendException(&pcFrame[0], EX_ILLEGAL_FUNCTION);
                break;
            }
            transmitDataMake(&pcFrame[0],len,ModbusRegister,SLAVE02_REG_COUNT);
            break;

        default : bad_Addrs++; break;                        //既非 01 也非 02：静默丢弃
    }
}    


// Modbus 侧数据处理
void uartDataHandler_Modbus(void)
{
    uint8_t len = frame_poll(FRAME_CH_RS485, (uint8_t *)rsFrame, NULL);
    if (len == 0) return;

    (void)mb_port_send(MB_PORT_PC, (const uint8_t *)rsFrame, len);   //将接收到的数据回显
    // printf_HXL(&rsFrame[0],len); //二进制格式回显
}


/* 异常响应：从机地址 + (功能码|0x80) + 异常码 + CRC */
void sendException(char *msg, uint8_t code)
{
    uint16_t crc;

    ModbusTx[0] = msg[0];
    ModbusTx[1] = (char)(msg[1] | 0x80);
    ModbusTx[2] = (char)code;

    crc = MODBUS_CRC16(ModbusTx, 3);
    ModbusTx[3] = (char)(crc & 0x00FF);
    ModbusTx[4] = (char)(crc >> 8);

    sendMessage(ModbusTx, 5);
}

/* ========================================================
 * 6. 具体功能码实现
 * ======================================================== */

/* 0x01 读线圈 */
// void makePacket_01(char *msg, uint8_t Lenght, const uint16_t *regs, uint16_t regCount)
// {
//     uint16_t RegAddress, NumberCoils, NumberByte, CRCValue;
//     uint8_t  i, ByteCount = 0;

//     RegAddress  = (uint16_t)((msg[2] << 8) | (uint8_t)msg[3]);
//     NumberCoils = (uint16_t)((msg[4] << 8) | (uint8_t)msg[5]);
//     NumberByte  = findByte((int16_t)NumberCoils);

//     /* 数量与地址合法性检查 */
//     if (NumberCoils == 0 || NumberCoils > 2000 ||
//         (uint32_t)RegAddress + NumberCoils > NUMBER_OF_COIL)
//     {
//         sendException(msg, (NumberCoils == 0) ? EX_ILLEGAL_VALUE : EX_ILLEGAL_ADDRESS);
//         return;
//     }

//     /* 编译期定长替代 VLA：全部线圈所需的字节数。
//      * 必须清零，否则 |= 会叠加到栈上垃圾值 */
//     uint8_t tempCoil[(NUMBER_OF_COIL + 7) / 8] = {0};

//     for (i = 0; i < NumberCoils; i++)
//     {
//         if ((i & 0x07) == 0 && i != 0)
//         {
//             ByteCount++;
//         }
//         if (ModbusCoil[RegAddress + i])
//         {
//             tempCoil[ByteCount] |= (uint8_t)(1 << (i & 0x07));
//         }
//     }

//     ModbusTx[0] = msg[0];
//     ModbusTx[1] = msg[1];
//     ModbusTx[2] = (char)NumberByte;

//     for (i = 0; i < NumberByte; i++)
//     {
//         ModbusTx[3 + i] = (char)tempCoil[i];
//     }

//     CRCValue = MODBUS_CRC16(ModbusTx, (uint8_t)(3 + NumberByte));
//     ModbusTx[3 + NumberByte] = (char)(CRCValue & 0x00FF);
//     ModbusTx[4 + NumberByte] = (char)(CRCValue >> 8);

//     sendMessage(ModbusTx, (uint8_t)(5 + NumberByte));
// }

/* 0x03 读保持寄存器 */
void makePacket_03(char *msg, uint8_t Lenght ,const uint16_t *regs, uint16_t regCount)
{
    uint16_t i;
    uint16_t RegAddress, NumberOfReg, CRCValue;

    RegAddress  = (uint16_t)((msg[2] << 8) | (uint8_t)msg[3]);
    NumberOfReg = (uint16_t)((msg[4] << 8) | (uint8_t)msg[5]);

    if (NumberOfReg == 0 || NumberOfReg > 125)
    {
        sendException(msg, EX_ILLEGAL_VALUE);
        return;
    }

    if ((uint32_t)RegAddress + NumberOfReg > regCount)
    {
        sendException(msg, EX_ILLEGAL_ADDRESS);
        return;
    }

    ModbusTx[0] = msg[0];
    ModbusTx[1] = msg[1];
    ModbusTx[2] = (char)(NumberOfReg * 2);        /* 字节数 */

    for (i = 0; i < NumberOfReg; i++)
    {
        /* 16 位寄存器拆成高字节在前、低字节在后 */
        ModbusTx[3 + i * 2] = (char)(regs[RegAddress + i] >> 8);
        ModbusTx[4 + i * 2] = (char)(regs[RegAddress + i] & 0x00FF);
    }

    CRCValue = MODBUS_CRC16(ModbusTx, (uint8_t)(3 + NumberOfReg * 2));
    ModbusTx[3 + NumberOfReg * 2] = (char)(CRCValue & 0x00FF);
    ModbusTx[4 + NumberOfReg * 2] = (char)(CRCValue >> 8);

    sendMessage(ModbusTx, (uint8_t)(5 + NumberOfReg * 2));
}

/* 0x05 写单个线圈 */
// void makePacket_05(char *msg, uint8_t Lenght, const uint16_t *regs, uint16_t regCount)
// {
//     uint16_t RegAddress, RegValue;

//     RegAddress = (uint16_t)((msg[2] << 8) | (uint8_t)msg[3]);
//     RegValue   = (uint16_t)((msg[4] << 8) | (uint8_t)msg[5]);

//     /* 规范只允许 0xFF00 (ON) 和 0x0000 (OFF) */
//     if (RegValue != 0xFF00 && RegValue != 0x0000)
//     {
//         sendException(msg, EX_ILLEGAL_VALUE);
//         return;
//     }

//     if (RegAddress >= NUMBER_OF_COIL)
//     {
//         sendException(msg, EX_ILLEGAL_ADDRESS);
//         return;
//     }

//     ModbusCoil[RegAddress] = (RegValue == 0xFF00);

//     sendMessage(msg, Lenght);        /* 正确响应 = 原样回显请求帧 */
// }

/* 0x06 写单个寄存器 */
// void makePacket_06(char *msg, uint8_t Lenght, const uint16_t *regs, uint16_t regCount)
// {
//     uint16_t RegAddress, RegValue;

//     RegAddress = (uint16_t)((msg[2] << 8) | (uint8_t)msg[3]);
//     RegValue   = (uint16_t)((msg[4] << 8) | (uint8_t)msg[5]);

//     if (RegAddress >= NUMBER_OF_REGISTER)
//     {
//         sendException(msg, EX_ILLEGAL_ADDRESS);
//         return;
//     }

//     ModbusRegister[RegAddress] = RegValue;

//     sendMessage(msg, Lenght);        /* 正确响应 = 原样回显请求帧 */
// }

/* 0x0F 写多个线圈 */
// void makePacket_15(char *msg, uint8_t Lenght, const uint16_t *regs, uint16_t regCount)
// {
//     uint16_t RegAddress, NumberOfCoils, CRCValue;
//     uint16_t i;

//     RegAddress    = (uint16_t)((msg[2] << 8) | (uint8_t)msg[3]);
//     NumberOfCoils = (uint16_t)((msg[4] << 8) | (uint8_t)msg[5]);

//     if (NumberOfCoils == 0 || NumberOfCoils > 1968 ||
//         (uint32_t)RegAddress + NumberOfCoils > NUMBER_OF_COIL)
//     {
//         sendException(msg, (NumberOfCoils == 0) ? EX_ILLEGAL_VALUE : EX_ILLEGAL_ADDRESS);
//         return;
//     }

//     for (i = 0; i < NumberOfCoils; i++)
//     {
//         /* 数据域从 msg[7] 开始，每 8 个线圈占 1 字节，低位在前 */
//         ModbusCoil[RegAddress + i] =
//             (msg[7 + (i >> 3)] & (1 << (i & 0x07))) ? true : false;
//     }

//     /* 正确响应：地址 + 功能码 + 起始地址 + 数量 + CRC (共 8 字节) */
//     memcpy(ModbusTx, msg, 6);

//     CRCValue = MODBUS_CRC16(ModbusTx, 6);
//     ModbusTx[6] = (char)(CRCValue & 0x00FF);
//     ModbusTx[7] = (char)(CRCValue >> 8);

//     sendMessage(ModbusTx, 8);
// }

/* 0x10 写多个寄存器 */
// void makePacket_16(char *msg, uint8_t Lenght, const uint16_t *regs, uint16_t regCount)
// {
//     uint16_t RegAddress, NumberOfReg, CRCValue, i;

//     HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);   /* 调试指示：收到 0x10 */

//     RegAddress  = (uint16_t)((msg[2] << 8) | (uint8_t)msg[3]);
//     NumberOfReg = (uint16_t)((msg[4] << 8) | (uint8_t)msg[5]);

//     if (NumberOfReg == 0 || NumberOfReg > 123 ||
//         (uint32_t)RegAddress + NumberOfReg > NUMBER_OF_REGISTER)
//     {
//         sendException(msg, (NumberOfReg == 0) ? EX_ILLEGAL_VALUE : EX_ILLEGAL_ADDRESS);
//         return;
//     }

//     for (i = 0; i < NumberOfReg; i++)
//     {
//         /* 数据域从 msg[7] 开始，每 2 字节一个大端寄存器 */
//         ModbusRegister[RegAddress + i] =
//             (uint16_t)(((uint16_t)(uint8_t)msg[7 + i * 2] << 8) |
//                         (uint8_t)msg[8 + i * 2]);
//     }

//     memcpy(ModbusTx, msg, 6);

//     CRCValue = MODBUS_CRC16(ModbusTx, 6);
//     ModbusTx[6] = (char)(CRCValue & 0x00FF);
//     ModbusTx[7] = (char)(CRCValue >> 8);

//     sendMessage(ModbusTx, 8);
// }

/* ========================================================
 * 7. 工具函数
 * ======================================================== */

/* 读取 NumberOfCoil 个线圈需要几个数据字节 */
uint8_t findByte(int16_t NumberOfCoil) 
{
    if (NumberOfCoil <= 0)
    {
        return 0;
    }
    return (uint8_t)((NumberOfCoil + 7) / 8);
}

/* Modbus CRC16，多项式 0xA001 (反向 0x8005)，初值 0xFFFF */
