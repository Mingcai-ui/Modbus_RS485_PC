/**
  * @file    dht11.c
  * @brief   DHT11 温湿度驱动实现
  *
  * 代码分层（由底向上，方便逐层阅读）：
  *   A. 引脚抽象层  —— 对 HAL 的一层薄封装，换芯片只改这一处
  *   B. 微秒计时层  —— 基于 DWT 时钟计数器，摆脱 HAL_Delay(ms) 的不够用
  *   C. 单总线位级层 —— 应答 + 40 位数据的紧贴时序读取（阻塞，约 4ms）
  *   D. 毫秒调度层  —— 状态机 + HAL_GetTick()，非阻塞驱动整体节奏
  *
  * 相对归档版的唯一改动：C 层的 40 位读取全程关中断（见 DHT11_ReadFrame）。
  */
#include "dht11.h"

/* ========================================================================== */
/* A. 引脚抽象层：所有对 GPIO 的操作统一走这里                                 */
/* ========================================================================== */

/* 使能引脚所在端口时钟（运行时按端口判断，通用 F1/F4/L4 等） */
static void DHT11_ClockEnable(void)
{
    if      (DHT11_GPIO_PORT == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); }
    else if (DHT11_GPIO_PORT == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
    else if (DHT11_GPIO_PORT == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
    else if (DHT11_GPIO_PORT == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
    else if (DHT11_GPIO_PORT == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
}

/* 把数据线切换为"推挽输出"，用于主机发送起始信号（可拉低） */
static void DHT11_SetOutput(void)
{
    GPIO_InitTypeDef io = {0};
    io.Pin   = DHT11_GPIO_PIN;
    io.Mode  = GPIO_MODE_OUTPUT_PP;   /* 推挽输出 */
    io.Pull  = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_GPIO_PORT, &io);
}

/* 把数据线切换为"输入 + 上拉"，即释放总线（上拉保证为高，DHT11 此时去向拉低） */
static void DHT11_SetInput(void)
{
    GPIO_InitTypeDef io = {0};
    io.Pin  = DHT11_GPIO_PIN;
    io.Mode = GPIO_MODE_INPUT;
    io.Pull = GPIO_PULLUP;            /* 内部上拉，替代部分外部上拉 */
    HAL_GPIO_Init(DHT11_GPIO_PORT, &io);
}

static void DHT11_WriteLow(void)
{
    HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_RESET);
}

static GPIO_PinState DHT11_Read(void)
{
    return HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN);
}

/* ========================================================================== */
/* B. 微秒计时层：DWT 时钟计数器，与主频关联，精确到微秒                       */
/* ========================================================================== */

/* 使能 DWT 计数器，必须在首次使用前调用一次 */
static void DHT11_DWT_Enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* 允许访问 DWT */
    DWT->CYCCNT  = 0;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;          /* 开启周期计数 */
}

/* 返回自开机以来的微秒数。
 * 说明：计数器会从 UINT32_MAX 翻转回 0，但所有调用方都用
 * "当前值 - 起始值"的 uint32 减法判断间隔，翻转被自动处理，无需担心。 */
static uint32_t DHT11_GetUs(void)
{
    static uint8_t dwt_inited = 0;
    if (!dwt_inited) {
        DHT11_DWT_Enable();
        dwt_inited = 1;
    }
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

/* 等待采集线上电平达到 target，超过 timeoutUs 未到则返回 0（超时失败） */
static uint8_t DHT11_WaitLevel(GPIO_PinState target, uint32_t timeoutUs)
{
    uint32_t start = DHT11_GetUs();
    while (DHT11_Read() != target) {
        if (DHT11_GetUs() - start > timeoutUs) { return 0; }
    }
    return 1;
}

/* 测量采集线"正在保持高电平"的时间（us），超时按最大值返回 */
static uint32_t DHT11_MeasureHighUs(uint32_t timeoutUs)
{
    uint32_t start = DHT11_GetUs();
    while (DHT11_Read() == GPIO_PIN_SET) {
        if (DHT11_GetUs() - start > timeoutUs) { return timeoutUs; }
    }
    return DHT11_GetUs() - start;
}

/* ========================================================================== */
/* C. 单总线位级层：应答 + 读取 40 位数据（阻塞，约 4ms，每 2 秒一次）          */
/* ========================================================================== */

#define DHT11_DATA_BYTES  5   /* 湿高/湿低/温高/温低/校验，共 5 字节 */

#if DHT11_DEBUG_TIMES
uint16_t DHT11_DbgBitUs[40];
uint8_t  DHT11_DbgRaw[5];
uint8_t  DHT11_DbgFailStage;   /* 最近一次失败的阶段，见下方 STAGE_* 定义 */
uint8_t  DHT11_DbgFailBit;     /* 失败发生在第几位（仅 bit 阶段有效） */

/* 失败阶段编号：定位「到底卡在哪一步」 */
#define STAGE_NONE        0    /* 未失败 */
#define STAGE_RESP_LOW    1    /* 释放总线后等不到 DHT11 拉低（从机没应答） */
#define STAGE_RESP_HIGH   2    /* 等不到应答低电平结束（应答被拉长或线被钳低） */
#define STAGE_BIT_HIGH    3    /* 40 位中某一位等不到上升沿（帧中途断了） */
#define STAGE_CHECKSUM    4    /* 40 位读全了，但校验和不匹配 */
#define STAGE_RESP_END    5    /* 应答高电平一直不结束（线被钳高或应答异常） */

#define DHT11_FAIL_STAGE(n)   (DHT11_DbgFailStage = (n))
#else
#define DHT11_FAIL_STAGE(n)   ((void)0)
#endif

/* 内层实现：原样保留归档版逻辑，所有失败路径直接 return。
 * 调用者负责关中断，本函数不关心。 */
static DHT11_Status DHT11_ReadFrame_Inner(DHT11_Handle_t *dht)
{
    uint8_t  bitBuf[DHT11_DATA_BYTES] = {0};   /* 40 位原始数据 */
    uint32_t i, time;

    DHT11_FAIL_STAGE(STAGE_NONE);

    /* 1. 等待 DHT11 应答，共三段：
     *      ① 从机拉低 80us  ② 从机拉高 80us  ③ 该高电平结束（下降沿）
     *    第 ③ 段容易被漏掉，漏了就会把应答脉冲当成第 0 位数据，整帧错位一位。 */
    if (!DHT11_WaitLevel(GPIO_PIN_RESET, DHT11_LEVEL_TIMEOUT_US))
    {
        DHT11_FAIL_STAGE(STAGE_RESP_LOW);
        return DHT11_ERROR;
    }
    if (!DHT11_WaitLevel(GPIO_PIN_SET,    DHT11_LEVEL_TIMEOUT_US))
    {
        DHT11_FAIL_STAGE(STAGE_RESP_HIGH);
        return DHT11_ERROR;
    }

    /* 必须再等这次应答高电平【结束】——这是 80us 应答脉冲的下降沿。
     *
     * 少了这一步会发生什么（2026-11-08 实测抓到的真实故障）：
     *   此时线还是高电平，下面循环里第一个 WaitLevel(SET) 会立刻返回、什么都不等，
     *   MeasureHighUs 就把这 80us 应答脉冲当成第 0 位数据（实测 86us > 40us 判为 '1'），
     *   于是整帧右移一位、第 40 位数据永远读不到，校验和自然对不上。
     *   表现是「约半数读成功但数值离谱」，例如 H=155.128%RH、T=14.3C。 */
    if (!DHT11_WaitLevel(GPIO_PIN_RESET, DHT11_LEVEL_TIMEOUT_US))
    {
        DHT11_FAIL_STAGE(STAGE_RESP_END);
        return DHT11_ERROR;
    }

    /* 2. 按位读取：每个 bit 都以"50us 低电平"起始，随后的高电平宽决定 0/1 */
    for (i = 0; i < 40; i++) {
        /* 先等完当前 bit 的低电平部分 */
        if (!DHT11_WaitLevel(GPIO_PIN_SET, DHT11_LEVEL_TIMEOUT_US))
        {
            DHT11_FAIL_STAGE(STAGE_BIT_HIGH);
#if DHT11_DEBUG_TIMES
            DHT11_DbgFailBit = (uint8_t)i;
#endif
            return DHT11_ERROR;
        }
        /* 测量高电平宽度：> 阈值判为 '1'，否则为 '0' */
        time = DHT11_MeasureHighUs(DHT11_LEVEL_TIMEOUT_US);
#if DHT11_DEBUG_TIMES
        DHT11_DbgBitUs[i] = (uint16_t)time;   /* 调试观测：宽度全 0 = 计时失效 */
#endif
        bitBuf[i / 8] <<= 1;
        if (time > DHT11_BIT_1_US) { bitBuf[i / 8] |= 0x01; }
    }

#if DHT11_DEBUG_TIMES
    /* 保存原始字节，便于和「换算后的温湿度」对照 */
    {
        uint32_t k;
        for (k = 0; k < DHT11_DATA_BYTES; k++) { DHT11_DbgRaw[k] = bitBuf[k]; }
    }
#endif

    /* 3. 校验：前 4 字节和（取低 8 位）应等于第 5 字节 */
    {
        uint8_t sum = (uint8_t)(bitBuf[0] + bitBuf[1] + bitBuf[2] + bitBuf[3]);
        if (sum != bitBuf[4])
        {
            DHT11_FAIL_STAGE(STAGE_CHECKSUM);
            return DHT11_ERROR;
        }
    }

    /* 4. 保存结果 */
    dht->humidity_int = bitBuf[0];         /* 湿度整数 */
    dht->humidity_dec = bitBuf[1];         /* 湿度小数 */
    dht->temp_int     = (int8_t)bitBuf[2]; /* 温度整数（带符号） */
    dht->temp_dec     = bitBuf[3];         /* 温度小数 */
    return DHT11_OK;
}

/* 外层：整段读取期间关中断，结束后按原样恢复。
 *
 * 为什么要关中断：
 *   单总线每个 bit 的可判窗口只有 26~70us。本工程开着 USART1(115200) 与
 *   USART2(9600) 两路接收中断，115200 下每 87us 就来一次，一次 ISR 就足以
 *   把 '0' 的高电平测成 '1'，或直接错过边沿导致整帧错位。
 *
 * 代价：
 *   最长约 4~5ms 不响应任何中断。此间若 PC 正在向 USART1 发帧，后续字节会因
 *   接收寄存器溢出（ORE）丢失，该帧作废。但 v1.0.0 已补齐两路错误恢复，
 *   链路不会卡死，下一帧可正常接收；Modbus 主站本身有超时重发机制。
 *
 * 计时为什么不受影响：
 *   DWT 是硬件周期计数器，与中断开关无关，关中断期间照常累加。
 *
 * 为什么用 PRIMASK 保存/恢复而不是直接 __enable_irq()：
 *   若本函数被外层临界区包裹，直接使能会提前打开中断、破坏外层保护。 */
static DHT11_Status DHT11_ReadFrame(DHT11_Handle_t *dht)
{
    uint32_t     primask = __get_PRIMASK();
    DHT11_Status st;

    __disable_irq();
    st = DHT11_ReadFrame_Inner(dht);
    __set_PRIMASK(primask);

    return st;
}

/* ========================================================================== */
/* D. 毫秒调度层：状态机 + HAL_GetTick()，非阻塞驱动整体节奏                  */
/* ========================================================================== */

/* 状态常量（与头文件注释对应）*/
#define DHT11_ST_WAIT        0   /* 等待读取间隔（2s） */
#define DHT11_ST_START_LOW   1   /* 起始信号：拉低中（20ms） */

void DHT11_Init(DHT11_Handle_t *dht)
{
    DHT11_GetUs();              /* 首次调用会顺带使能 DWT */
    DHT11_ClockEnable();
    DHT11_SetInput();           /* 初始处于输入上拉（总线释放态） */
    dht->state     = DHT11_ST_WAIT;
    dht->last_tick = HAL_GetTick();
}

DHT11_Status DHT11_Handler(DHT11_Handle_t *dht)
{
    uint32_t now = HAL_GetTick();

    switch (dht->state)
    {
    case DHT11_ST_WAIT:                       /* 等待满 2s 读取间隔 */
        if (now - dht->last_tick >= DHT11_READ_INTERVAL_MS) {
            DHT11_SetOutput();                /* 进入输出，准备拉低 */
            DHT11_WriteLow();                 /* 起始信号：开始拉低 */
            dht->last_tick = now;             /* 记录拉低起始时刻 */
            dht->state = DHT11_ST_START_LOW;
        }
        return DHT11_BUSY;

    case DHT11_ST_START_LOW:                  /* 起始信号拉低中（20ms） */
        if (now - dht->last_tick >= DHT11_START_LOW_MS) {
            dht->state     = DHT11_ST_WAIT;   /* 回到等待，准备下一次间隔计时 */
            dht->last_tick = now;
            DHT11_SetInput();                 /* 释放总线：从机开始应答 */
            return DHT11_ReadFrame(dht);      /* 紧贴时序读取，返回最终结果 */
        }
        return DHT11_BUSY;

    default:
        return DHT11_ERROR;
    }
}

/* ========================================================================== */
/* 结果读取                                                                   */
/* ========================================================================== */

float DHT11_GetTemp(DHT11_Handle_t *dht)
{
    return (float)dht->temp_int + (float)dht->temp_dec / 10.0f;
}

float DHT11_GetHumi(DHT11_Handle_t *dht)
{
    return (float)dht->humidity_int + (float)dht->humidity_dec / 10.0f;
}
