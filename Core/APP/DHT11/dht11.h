/**
  * @file    dht11.h
  * @brief   DHT11 温湿度传感器驱动（单总线 / 非阻塞调度版）
  *
  * 移植自「可移植模块代码/DHT11」，适配 Modbus_RS485_PC 工程：
  *   - 引脚沿用 PA11（本工程 PA1=RS485 DE、PA2/PA3=USART2、PA9/PA10=USART1、
  *     PA13/PA14=SWD、PC13=LED，PA11 空闲，无需改 CubeMX）
  *   - DHT11_ReadFrame() 增加关中断保护（本工程有 USART1/USART2 两路接收中断，
  *     115200 下每 87us 一次，会破坏单总线的微秒级时序）
  *
  * 设计要点（可读性 / 可移植性 / 非阻塞）：
  *
  * 1. 移植只需要改"可移植配置区"：引脚、端口、时序参数。
  *    GPIO 的读写、输入/输出切换被封装成内部小函数，
  *    上层业务逻辑（状态机、校验）不直接触碰 HAL，方便换芯片。
  *
  * 2. 时序分两类处理（这是 DHT11 的客观限制，先理解再动手）：
  *    - 毫秒级（起始信号 18ms、读取间隔 2s）→ 状态机 + HAL_GetTick()，非阻塞；
  *    - 微秒级（应答 + 40 位数据）→ 每个 bit 只有 26~70us，必须紧贴时序读取，
  *      无法真正非阻塞。这里集中在一次约 4ms 的阻塞读取里，且每 2 秒仅一次。
  *    结论：主循环其余时间不会被长时间卡住，这是单总线传感器的合理折中。
  *
  * 使用示例（放入主循环轮询即可）：
  * @code
  *   DHT11_Handle_t dht;
  *   DHT11_Init(&dht);
  *   while(1){
  *       DHT11_Status st = DHT11_Handler(&dht);   // 反复调用，内部自动调度
  *       if(st == DHT11_OK){
  *           float t = DHT11_GetTemp(&dht);
  *           float h = DHT11_GetHumi(&dht);
  *           // ... 每 2 秒拿到一次新数据
  *       }
  *   }
  * @endcode
  */
#ifndef __DHT11_H__
#define __DHT11_H__

#include "main.h"   /* HAL 以及 SystemCoreClock */

/* ========================================================================== */
/* ① 可移植配置区：换芯片/换引脚只改这里                                     */
/* ========================================================================== */

/* 数据引脚：接端口 + 引脚。这里默认 PA11，可用编译器宏覆盖 */
#ifndef DHT11_GPIO_PORT
  #define DHT11_GPIO_PORT       GPIOA
#endif
#ifndef DHT11_GPIO_PIN
  #define DHT11_GPIO_PIN        GPIO_PIN_11
#endif

/* 起始信号低电平持续时间（ms）：DHT11 要求 >= 18ms，留余量取 20 */
#ifndef DHT11_START_LOW_MS
  #define DHT11_START_LOW_MS    20U
#endif

/* 两次读取最小间隔（ms）：DHT11 手册要求 >= 1000ms，留余量取 2000 */
#ifndef DHT11_READ_INTERVAL_MS
  #define DHT11_READ_INTERVAL_MS  2000U
#endif

/* 判定 bit 为 1 的高电平阈值（us）：DHT11 中 '0' 高电平 26~28us，'1' 约 70us */
#ifndef DHT11_BIT_1_US
  #define DHT11_BIT_1_US        40U
#endif

/* 等待采集线上某种电平的最长超时（us） */
#ifndef DHT11_LEVEL_TIMEOUT_US
  #define DHT11_LEVEL_TIMEOUT_US  200U
#endif

/* 调试观测：记录每位实测高电平宽度 + 原始 5 字节。
 * 用途：定位「读回 0.0℃ / 0.0%RH 但 ok 计数在涨」这类问题——
 *       若宽度全是 0，说明微秒计时（DWT）没在跑；若宽度是 26/70 交替，
 *       说明时序正常、问题在传感器本身或校验逻辑。
 * 验证通过后改成 0 即可，不影响任何功能代码。 */
#ifndef DHT11_DEBUG_TIMES
  #define DHT11_DEBUG_TIMES   0
#endif

#if DHT11_DEBUG_TIMES
extern uint16_t DHT11_DbgBitUs[40];   /* 40 位各自的高电平宽度（us） */
extern uint8_t  DHT11_DbgRaw[5];      /* 最近一次读到的原始 5 字节 */
extern uint8_t  DHT11_DbgFailStage;   /* 失败阶段：1=等应答低 2=等应答高 3=等某位高 4=校验 */
extern uint8_t  DHT11_DbgFailBit;     /* 失败时具体卡在第几位（0~39） */
#endif

/* ========================================================================== */
/* ② 对外接口                                                                */
/* ========================================================================== */

/* 每次调用 DHT11_Handler() 的返回状态 */
typedef enum {
    DHT11_BUSY   = 0,   /* 正在调度中（等待间隔 / 起始信号进行中），暂无新数据 */
    DHT11_OK     = 1,   /* 本次调用完成了一次成功读取，新数据已就绪 */
    DHT11_ERROR  = 2,   /* 读取失败：无响应（超时）或校验和不匹配 */
} DHT11_Status;

/* 温湿度结果句柄：一个传感器对应一个实例 */
typedef struct {
    /* --- 结果数据（读取成功后被更新）--- */
    uint8_t  humidity_int;   /* 湿度整数部分（0~100 %RH） */
    uint8_t  humidity_dec;   /* 湿度小数部分 */
    int8_t   temp_int;       /* 温度整数部分（DHT11 为 0~50 ℃，DHT22 才有负温） */
    uint8_t  temp_dec;       /* 温度小数部分 */

    /* --- 内部状态（仅供驱动使用，用户勿动）--- */
    uint8_t  state;          /* 状态机状态：0=等待间隔, 1=起始信号（含读取） */
    uint32_t last_tick;      /* 最近一次时间戳（ms） */
} DHT11_Handle_t;

/* 初始化：复位状态机 + 使能微秒计时器（DWT）。主循环前调用一次。
 * 注意：必须在 SystemClock_Config() 之后调用，否则 SystemCoreClock 不是 72MHz，
 *       DWT 换算出的微秒值会整体偏大/偏小。 */
void DHT11_Init(DHT11_Handle_t *dht);

/* 非阻塞处理函数：放入主循环反复调用。
 *   返回 DHT11_OK 表示本次刚完成一次成功读取。 */
DHT11_Status DHT11_Handler(DHT11_Handle_t *dht);

/* 读取成功后取结果（单位 ℃ / %RH），内部做了合并 */
float DHT11_GetTemp(DHT11_Handle_t *dht);
float DHT11_GetHumi(DHT11_Handle_t *dht);

#endif /* __DHT11_H__ */
