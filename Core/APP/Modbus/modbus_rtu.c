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
#include "UART_Buf.h"
#include "usart.h"
/* ================== 全局数据区 ================== */
uint16_t ModbusRegister[NUMBER_OF_REGISTER];
bool     ModbusCoil[NUMBER_OF_COIL];

/* ================== 接收状态 ================== */
volatile uint8_t RxInterruptFlag;               //UART2数据接收状态标志位
volatile uint8_t uartTimeCounter;               //UART2数据间隔计数器
volatile uint8_t uartPacketComplatedFlag;       //UART2收到完整帧数据标志位

volatile uint8_t RxInterruptFlag_PC;            //UART1数据接收状态标志位
volatile uint8_t uartTimeCounter_PC;            //UART1数据间隔计数器
volatile uint8_t uartPacketComplatedFlag_PC;    //UART1收到完整帧数据标志位


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
 * RS485 方向控制
 *   PA1 = 1 : DE 有效，发送模式
 *   PA1 = 0 : RE 有效，接收模式
 * 注意：MAX485 类收发器从发送切回接收的瞬间会把总线拉低，
 *       必须等最后一个字节的停止位完全移出 (TC 置位) 再切换。
 * ======================================================== */
static void rs485Mode(uint8_t tx)
{
    HAL_GPIO_WritePin(RS485_GPIO_Port, RS485_Pin, tx ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ========================================================
 * 2. 功能码分发
 * ======================================================== */
void transmitDataMake(char *msg, uint8_t Lenght)
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
    case ReadCoil:               makePacket_01(msg, Lenght); break;
    case ReadHoldingRegister:    makePacket_03(msg, Lenght); break;
    case WriteSingleRegister:    makePacket_06(msg, Lenght); break;
    case WriteSingleCoil:        makePacket_05(msg, Lenght); break;
    case WriteMultipleCoils:     makePacket_15(msg, Lenght); break;
    case WriteMultipleRegisters: makePacket_16(msg, Lenght); break;

    default:
        sendException(msg, EX_ILLEGAL_FUNCTION);   /* 不支持的功能码 → 0x81 */
        break;
    }
}

/* ========================================================
 * 3. 数据解析与处理 (main 的 while(1) 里轮询)
 * ======================================================== */
void uartDataHandler(void)
{
    uint8_t  tempCounter;
    uint16_t CRCValue;

    if (uartPacketComplatedFlag != SET)   /* 还没有完整帧 */
    {
        return;
    }

    uartPacketComplatedFlag = RESET;
    tempCounter = DataCounter_Modbus;            /* 先取真实帧长 */
    DataCounter_Modbus = 0;

    /* 最短合法帧 = 从机地址 + 功能码 + CRC(2) = 4 字节。
     * 少于 4 字节说明是噪声或超长帧残片，直接丢弃。
     * 这一句同时挡住了 tempCounter-2 在 uint8_t 下的下溢 (0-2 → 254)。 */
    if (tempCounter < 4)
    {
        memset(ModbusRx, 0, BUFFERSIZE);
        return;
    }

    memcpy(tempModbusRx, ModbusRx, tempCounter);   /* 先拷出，再清接收区 */
    memset(ModbusRx, 0, BUFFERSIZE);
    memset(ModbusTx, 0, BUFFERSIZE);

    /* CRC 校验：对前 tempCounter-2 字节计算，与帧尾两字节比对 */
    CRCValue = MODBUS_CRC16(tempModbusRx, tempCounter - 2);
    /* Modbus 规定 CRC 低字节在前、高字节在后 */
    rxCRC = ((uint8_t)tempModbusRx[tempCounter - 1] << 8) |
             (uint8_t)tempModbusRx[tempCounter - 2];

    if (rxCRC != CRCValue)
    {
        return;                            /* CRC 错，静默丢弃 */
    }

    if (tempModbusRx[0] != SLAVEID)
    {
        return;                            /* 不是发给本机的（含广播 0x00） */
    }
    transmitDataMake(&tempModbusRx[0], tempCounter);
}

/* ========================================================
 * 4. 帧结束判断 (SysTick 中断里每 1ms 调用一次)
 * ======================================================== */
void uartTimer(void)
{
    if (RxInterruptFlag != SET)
    {
        return;
    }

    if (++uartTimeCounter >= FRAME_GAP_MS)   /* 静默够久，认为一帧结束 */
    {
        RxInterruptFlag         = RESET;
        uartTimeCounter         = 0;
        uartPacketComplatedFlag = SET;      //取到完整的一帧数据
    }
}
void uartTimer_PC(void)
{
    if (RxInterruptFlag_PC != SET)
    {
        return;
    }

    if (++uartTimeCounter_PC >= FRAME_GAP_MS)   /* 静默够久，认为一帧结束 */
    {
        RxInterruptFlag_PC         = RESET;
        uartTimeCounter_PC         = 0;
        uartPacketComplatedFlag_PC = SET;      //取到完整的一帧数据
    }
}

/* ========================================================
 * 5. 发送：按从机地址决定出口
 *
 * 帧头第一个字节就是从机地址，它同时决定了这帧该往哪条链路走：
 *   0x01 → RS485 总线上的外部变送器，写 USART2 并拉高 DE
 *   其他 → 本机自己产生的应答（如 0x02 的 DHT11），直接写 USART1 回 PC
 *
 * 放在这里统一判断，所有 makePacket_* 和 sendException 都不用关心出口，
 * 调用方式和以前完全一样。
 *
 * RS485 侧：HAL_UART_Transmit 返回时 TC 标志已置位，最后一个字节已移出，
 * 此时拉低 DE 不会截断数据，也不会与总线冲突。
 * ======================================================== */
void sendMessage(char *msg, uint8_t len)
{
    if ((uint8_t)msg[0] == SLAVEID)
    {
        rs485Mode(1);
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, len, (uint32_t)len*2+10);
        rs485Mode(0);
    }
    else
    {
        /* 本机应答：USART1 全双工，不需要方向切换 */
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, (uint32_t)len*2+10);
    }
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
    if(uartPacketComplatedFlag_PC != SET) return;

    uartPacketComplatedFlag_PC=RESET;       //完整帧标志位清零
    uint8_t len = DataCounter_PC;
    DataCounter_PC=0;             //长度指针清零

    /* 超长标志先取走再清零：无论本帧后面怎么走，都不能残留到下一帧 */
    uint8_t overflowed = frameOverflow_PC;
    frameOverflow_PC = RESET;

    if(len == 0) return;

    memcpy(pcFrame,PC_RX,len);    //拷贝PC发送的数据
    memset(PC_RX,0,BUFFERSIZE);   //UART1缓冲区清空

    /* 把"超长"折进同一个判决变量，下面只需要一处分支。
       超长的证据只存在于中断里，所以由标记转成枚举值。 */
    FrameCheckResult verdict = overflowed ? FRAME_Too_LONG : checkFrame(pcFrame, len);

    if(verdict != FRAME_OK) //判断帧是否合法
    {
        if      (verdict == FRAME_Too_SHORT) { bad_SHORT++; }
        else if (verdict == FRAME_Too_LONG)  { bad_LONG++;  }
        else if (verdict == FRAME_CRC_ERR)   { bad_CRC++;   }
        return;
    }

    switch((uint8_t)pcFrame[0])
    {
        /* 0x01：RS485 总线上的外部温湿度变送器。原样透传，由它自己应答；
           它的应答经 USART2 回来后由 uartDataHandler_Modbus 回显给 PC。 */
        case 0x01: sendMessage(&pcFrame[0],len);    break;

        /* 0x02：本机 DHT11 虚拟从机。数据源是 ModbusRegister[0..1]
           （main 主循环周期采样填充），只开放读。
           写操作在这里没有意义：写进去的值会被下一次 2s 采样直接覆盖，
           反而让 PC 端误以为写入成功。所以非读功能码一律回异常 0x01。

           放行后交给标准功能码分发器，makePacket_03 末尾调 sendMessage，
           该函数按地址判断出口，0x02 会从 USART1 直接回 PC，
           不会打到 RS485 总线上。寄存器越界则由 makePacket_03 回异常 0x02，
           两种异常同样从 USART1 回 PC。

           注意：目前只支持 0x03 读保持寄存器。若以后要用 0x04 读输入寄存器，
           需要先在 transmitDataMake 里补 makePacket_04。 */
        case 0x02:
            if ((uint8_t)pcFrame[1] != ReadHoldingRegister)
            {
                sendException(&pcFrame[0], EX_ILLEGAL_FUNCTION);
                break;
            }
            transmitDataMake(&pcFrame[0],len);
            break;

        default : bad_Addrs++; break;                        //既非 01 也非 02：静默丢弃
    }
}    


// Modbus 侧数据处理
void uartDataHandler_Modbus(void)
{
    if(uartPacketComplatedFlag != SET) return;
    uartPacketComplatedFlag=RESET;      //完整帧标志位清零

    uint8_t len = DataCounter_Modbus;   //取出长度
    DataCounter_Modbus = 0;             //长度指针清零

    if(len == 0) return;
    memcpy(rsFrame,ModbusRx,len);
    memset(ModbusRx,0,BUFFERSIZE);  //UART2缓冲区清空
    HAL_UART_Transmit(&huart1, (uint8_t *)rsFrame, len, (uint32_t)len*2+10);   //将接收到的数据回显
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
void makePacket_01(char *msg, uint8_t Lenght)
{
    uint16_t RegAddress, NumberCoils, NumberByte, CRCValue;
    uint8_t  i, ByteCount = 0;

    RegAddress  = (uint16_t)((msg[2] << 8) | (uint8_t)msg[3]);
    NumberCoils = (uint16_t)((msg[4] << 8) | (uint8_t)msg[5]);
    NumberByte  = findByte((int16_t)NumberCoils);

    /* 数量与地址合法性检查 */
    if (NumberCoils == 0 || NumberCoils > 2000 ||
        (uint32_t)RegAddress + NumberCoils > NUMBER_OF_COIL)
    {
        sendException(msg, (NumberCoils == 0) ? EX_ILLEGAL_VALUE : EX_ILLEGAL_ADDRESS);
        return;
    }

    /* 编译期定长替代 VLA：全部线圈所需的字节数。
     * 必须清零，否则 |= 会叠加到栈上垃圾值 */
    uint8_t tempCoil[(NUMBER_OF_COIL + 7) / 8] = {0};

    for (i = 0; i < NumberCoils; i++)
    {
        if ((i & 0x07) == 0 && i != 0)
        {
            ByteCount++;
        }
        if (ModbusCoil[RegAddress + i])
        {
            tempCoil[ByteCount] |= (uint8_t)(1 << (i & 0x07));
        }
    }

    ModbusTx[0] = msg[0];
    ModbusTx[1] = msg[1];
    ModbusTx[2] = (char)NumberByte;

    for (i = 0; i < NumberByte; i++)
    {
        ModbusTx[3 + i] = (char)tempCoil[i];
    }

    CRCValue = MODBUS_CRC16(ModbusTx, (uint8_t)(3 + NumberByte));
    ModbusTx[3 + NumberByte] = (char)(CRCValue & 0x00FF);
    ModbusTx[4 + NumberByte] = (char)(CRCValue >> 8);

    sendMessage(ModbusTx, (uint8_t)(5 + NumberByte));
}

/* 0x03 读保持寄存器 */
void makePacket_03(char *msg, uint8_t Lenght)
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

    if ((uint32_t)RegAddress + NumberOfReg > NUMBER_OF_REGISTER)
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
        ModbusTx[3 + i * 2] = (char)(ModbusRegister[RegAddress + i] >> 8);
        ModbusTx[4 + i * 2] = (char)(ModbusRegister[RegAddress + i] & 0x00FF);
    }

    CRCValue = MODBUS_CRC16(ModbusTx, (uint8_t)(3 + NumberOfReg * 2));
    ModbusTx[3 + NumberOfReg * 2] = (char)(CRCValue & 0x00FF);
    ModbusTx[4 + NumberOfReg * 2] = (char)(CRCValue >> 8);

    sendMessage(ModbusTx, (uint8_t)(5 + NumberOfReg * 2));
}

/* 0x05 写单个线圈 */
void makePacket_05(char *msg, uint8_t Lenght)
{
    uint16_t RegAddress, RegValue;

    RegAddress = (uint16_t)((msg[2] << 8) | (uint8_t)msg[3]);
    RegValue   = (uint16_t)((msg[4] << 8) | (uint8_t)msg[5]);

    /* 规范只允许 0xFF00 (ON) 和 0x0000 (OFF) */
    if (RegValue != 0xFF00 && RegValue != 0x0000)
    {
        sendException(msg, EX_ILLEGAL_VALUE);
        return;
    }

    if (RegAddress >= NUMBER_OF_COIL)
    {
        sendException(msg, EX_ILLEGAL_ADDRESS);
        return;
    }

    ModbusCoil[RegAddress] = (RegValue == 0xFF00);

    sendMessage(msg, Lenght);        /* 正确响应 = 原样回显请求帧 */
}

/* 0x06 写单个寄存器 */
void makePacket_06(char *msg, uint8_t Lenght)
{
    uint16_t RegAddress, RegValue;

    RegAddress = (uint16_t)((msg[2] << 8) | (uint8_t)msg[3]);
    RegValue   = (uint16_t)((msg[4] << 8) | (uint8_t)msg[5]);

    if (RegAddress >= NUMBER_OF_REGISTER)
    {
        sendException(msg, EX_ILLEGAL_ADDRESS);
        return;
    }

    ModbusRegister[RegAddress] = RegValue;

    sendMessage(msg, Lenght);        /* 正确响应 = 原样回显请求帧 */
}

/* 0x0F 写多个线圈 */
void makePacket_15(char *msg, uint8_t Lenght)
{
    uint16_t RegAddress, NumberOfCoils, CRCValue;
    uint16_t i;

    RegAddress    = (uint16_t)((msg[2] << 8) | (uint8_t)msg[3]);
    NumberOfCoils = (uint16_t)((msg[4] << 8) | (uint8_t)msg[5]);

    if (NumberOfCoils == 0 || NumberOfCoils > 1968 ||
        (uint32_t)RegAddress + NumberOfCoils > NUMBER_OF_COIL)
    {
        sendException(msg, (NumberOfCoils == 0) ? EX_ILLEGAL_VALUE : EX_ILLEGAL_ADDRESS);
        return;
    }

    for (i = 0; i < NumberOfCoils; i++)
    {
        /* 数据域从 msg[7] 开始，每 8 个线圈占 1 字节，低位在前 */
        ModbusCoil[RegAddress + i] =
            (msg[7 + (i >> 3)] & (1 << (i & 0x07))) ? true : false;
    }

    /* 正确响应：地址 + 功能码 + 起始地址 + 数量 + CRC (共 8 字节) */
    memcpy(ModbusTx, msg, 6);

    CRCValue = MODBUS_CRC16(ModbusTx, 6);
    ModbusTx[6] = (char)(CRCValue & 0x00FF);
    ModbusTx[7] = (char)(CRCValue >> 8);

    sendMessage(ModbusTx, 8);
}

/* 0x10 写多个寄存器 */
void makePacket_16(char *msg, uint8_t Lenght)
{
    uint16_t RegAddress, NumberOfReg, CRCValue, i;

    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);   /* 调试指示：收到 0x10 */

    RegAddress  = (uint16_t)((msg[2] << 8) | (uint8_t)msg[3]);
    NumberOfReg = (uint16_t)((msg[4] << 8) | (uint8_t)msg[5]);

    if (NumberOfReg == 0 || NumberOfReg > 123 ||
        (uint32_t)RegAddress + NumberOfReg > NUMBER_OF_REGISTER)
    {
        sendException(msg, (NumberOfReg == 0) ? EX_ILLEGAL_VALUE : EX_ILLEGAL_ADDRESS);
        return;
    }

    for (i = 0; i < NumberOfReg; i++)
    {
        /* 数据域从 msg[7] 开始，每 2 字节一个大端寄存器 */
        ModbusRegister[RegAddress + i] =
            (uint16_t)(((uint16_t)(uint8_t)msg[7 + i * 2] << 8) |
                        (uint8_t)msg[8 + i * 2]);
    }

    memcpy(ModbusTx, msg, 6);

    CRCValue = MODBUS_CRC16(ModbusTx, 6);
    ModbusTx[6] = (char)(CRCValue & 0x00FF);
    ModbusTx[7] = (char)(CRCValue >> 8);

    sendMessage(ModbusTx, 8);
}

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

//打印应答帧数据工具
void printf_HXL(const char *msg, uint8_t Lenght)
{
    printf("ModbusTx:");
    for (uint8_t i = 0; i < Lenght; i++)
    {
        printf("%02X ", (uint8_t) msg[i]);
    }
    printf("\r\n");
}

/* Modbus CRC16，多项式 0xA001 (反向 0x8005)，初值 0xFFFF */
