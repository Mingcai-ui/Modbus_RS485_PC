/**
  * @file    dht11.c
  * @brief   DHT11 温湿度驱动实现（单总线 / 非阻塞调度版）
  *
  * ==========================================================================
  * 使用方法（三步，塞进主循环就完事）
  * ==========================================================================
  *
  * ① 定义句柄 + 初始化（全程只做一次，位置有讲究）
  *      #include "dht11.h"
  *      DHT11_Handle_t dht;
  *      DHT11_Init(&dht);
  *    必须放在 SystemClock_Config() 之后：DWT 换算微秒要拿 SystemCoreClock 当除数，
  *    此时若主频还没切到 72MHz，算出来的"微秒"就是错的，整帧位宽全部偏大/偏小。
  *
  * ② 在主循环里反复调用（不要自己加延时，驱动内部自带 2s 节拍）
  *      DHT11_Status st = DHT11_Handler(&dht);
  *
  * ③ 只有返回 DHT11_OK 的那一次，数据才是刚读到的
  *      if (st == DHT11_OK) {
  *          float t = DHT11_GetTemp(&dht);   // 单位 ℃
  *          float h = DHT11_GetHumi(&dht);   // 单位 %RH
  *      }
  *
  * 返回值语义：
  *   DHT11_BUSY  —— 还在等（等 2s 间隔 / 起始信号进行中）。日常绝大多数调用都是它，
  *                  不是错误，继续调用即可。
  *   DHT11_OK    —— 本次调用刚读完一整帧且校验通过，dht 里的数据已刷新。
  *   DHT11_ERROR —— 本次失败（无应答超时 或 校验和不匹配）。注意：失败时 dht 里
  *                  保留的是上一次成功的数据，不会被写成 0，所以界面上看到的是
  *                  "旧值"而不是"坏值"，配合错误计数用更清楚。
  *
  * 调用约束（这几条都是踩过的，别省）：
  *   1. Init 必须在 SystemClock_Config() 之后（见 ①）。
  *   2. 不要在中断 / 定时器回调 / FreeRTOS 软件定时器回调里调用：读一帧要关中断
  *      约 4~5ms，等于把整个系统的中断响应按下去 4ms，放中断里会连锁丢事件。
  *   3. 主循环里不要自己加 HAL_Delay 控制频率，节拍在驱动内部的 2s 间隔上；
  *      调用越勤，读到的数据越"及时"（数据仍然是每 2s 才换一次）。
  *   4. 关中断那 4~5ms 内 USART 接收中断进不来，若此刻 PC 正在发帧，后续字节会因
  *      接收寄存器溢出（ORE）丢掉、该帧作废（链路不会卡死，靠错误回调恢复 + 主站重发）。
  *      所以本工程的设计意图是：只在两条总线都静默时才驱动传感器。
  *
  * 本工程接入现状（2026-11-11 核对）：
  *   main.c 只做了 #include "dht11.h" 并留了一个 DHT11_LOG_ENABLE 开关，
  *   while(1) 里还没有调用 DHT11_Init / DHT11_Handler —— 也就是说驱动已写好、
  *   上板验证过，但当前还没接进主循环。要接的时候按 ①②③ 三步插入即可。
  *
  * 移植：只改 dht11.h 顶部「① 可移植配置区」里的引脚和宏，本文件一行都不用动。
  * 详细设计说明与实测记录见同目录 DHT11_介绍.md。
  *
  * ==========================================================================
  * 代码分层（由底向上，方便逐层阅读）
  * ==========================================================================
  *   A. 引脚抽象层  —— 对 HAL 的一层薄封装，换芯片只改这一处
  *   B. 微秒计时层  —— 基于 DWT 时钟计数器，摆脱 HAL_Delay(ms) 的不够用
  *   C. 单总线位级层 —— 应答 + 40 位数据的紧贴时序读取（阻塞，约 4ms）
  *   D. 毫秒调度层  —— 状态机 + HAL_GetTick()，非阻塞驱动整体节奏
  *
  * 相对归档版的唯一功能改动：C 层的 40 位读取全程关中断（见 DHT11_ReadFrame）。
  */
#include "dht11.h"

/* ========================================================================== */
/* A. 引脚抽象层：所有对 GPIO 的操作统一走这里                                 */
/* ========================================================================== */

/* 使能引脚所在端口时钟。
 *
 * 为什么不直接在 CubeMX 里配好、这里不管：
 *   那样这个驱动就和具体工程绑死了，换到别的板子（引脚换到 GPIOB/C）就要回去改
 *   CubeMX 重新生成。这里用"运行时比较端口指针"的办法，把时钟使能也一起搬进
 *   驱动内部，配置区只填 PORT/PIN 两个宏就能整体搬走。
 *
 * 用 if-else 链而不是开关表：F1 的 RCC 使能宏是带参数的宏（不是常量），
 * 放不进数组；六个分支编译出来也就几条比较指令，调用频率极低（只在 Init 一次），
 * 完全不值得为"优雅"去搞函数指针表。
 *
 * 注意：这里只覆盖 GPIOA~E。若把引脚挪到 GPIOF/G，需要按同样写法补一行。 */
static void DHT11_ClockEnable(void)
{
    if      (DHT11_GPIO_PORT == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); }
    else if (DHT11_GPIO_PORT == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
    else if (DHT11_GPIO_PORT == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
    else if (DHT11_GPIO_PORT == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
    else if (DHT11_GPIO_PORT == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
}

/* 把数据线切换为"推挽输出"，用于主机发送起始信号（可拉低）。
 *
 * 单总线的本质矛盾：一根线既要主机发命令（输出），又要主机收数据（输入），
 * 所以每次方向切换都要重配 GPIO。这也是本层存在的理由——上层只管"要输出还是要
 * 输入"，不用关心 GPIO_InitTypeDef 怎么写。
 *
 * 为什么用推挽（OUTPUT_PP）而不是开漏（OUTPUT_OD）：
 *   这里只需要把线拉低 20ms，推挽输出阻抗低、拉低干净利落；开漏要靠外部上拉拉高，
 *   本板依赖的是内部弱上拉，边沿会变慢。DHT11 对起始信号的低电平宽度不苛刻，
 *   但推挽留给时序的余量更大，属于"选更硬的那个"。
 *
 * 为什么 Speed 给 HIGH：2MHz 档的上升/下降沿更缓，20ms 的低电平不在乎这个，
 *   但同一条线随后要用来读 26~70us 的脉冲，切换成输入时边沿特性继承端口的
 *   输出驱动设置，给 HIGH 更安全。 */
static void DHT11_SetOutput(void)
{
    GPIO_InitTypeDef io = {0};
    io.Pin   = DHT11_GPIO_PIN;
    io.Mode  = GPIO_MODE_OUTPUT_PP;   /* 推挽输出 */
    io.Pull  = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_GPIO_PORT, &io);
}

/* 把数据线切换为"输入 + 上拉"，即释放总线。
 *
 * "释放"这个词要理解准确：不是把线驱动成高电平，而是把引脚变成高阻态、让上拉
 * 电阻把线"托"在高电平。差别在于——如果主机自己驱动高电平，那么 DHT11 同时间
 * 拉低时就是两个输出对顶（短路风险 + 电平不确定）；改成高阻后，谁拉低就是谁说了算，
 * 这正是单总线能双向通信的前提。
 *
 * 内部上拉约 30~50kΩ，RC 时间常数偏大，长线或干扰环境下边沿会肉。
 * 所以硬件上建议再并一个 4.7k~10k 的外部上拉，代码这条内部上拉当兜底。
 *
 * 注意：本函数在起始信号结束（释放总线）和 Init 时都会被调用，是"空闲态"的正确姿势。 */
static void DHT11_SetInput(void)
{
    GPIO_InitTypeDef io = {0};
    io.Pin  = DHT11_GPIO_PIN;
    io.Mode = GPIO_MODE_INPUT;
    io.Pull = GPIO_PULLUP;            /* 内部上拉，替代部分外部上拉 */
    HAL_GPIO_Init(DHT11_GPIO_PORT, &io);
}

/* 拉低数据线（唯一的"主机说话"动作）：
 * 起始信号就是"拉低 ≥18ms 再释放"，本驱动取 20ms 留余量，
 * 20ms 的定时不由本函数负责，而是交给 D 层的状态机（这样才不阻塞）。 */
static void DHT11_WriteLow(void)
{
    HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_RESET);
}

/* 读当前电平。返回 GPIO_PinState（SET/RESET）而不是 0/1：
 * 保持和 HAL 同一套类型，调用处的比较写起来是 DHT11_Read() == GPIO_PIN_SET，
 * 语义自解释，不必在中间再转一次。 */
static GPIO_PinState DHT11_Read(void)
{
    return HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN);
}

/* ========================================================================== */
/* B. 微秒计时层：DWT 时钟计数器，与主频关联，精确到微秒                       */
/* ========================================================================== */

/* 使能 DWT 计数器，必须在首次使用前调用一次。
 *
 * 为什么不能用 HAL_GetTick() 代替：它的分辨率是 1ms，而 DHT11 每个 bit 的高电平
 * 只有 26us(0) 或 70us(1)，量级差 30 倍，根本分不出来。
 * 也不能用定时器（TIMx）来做：那要占一个硬件资源，还要配 CubeMX，
 * 而 Cortex-M3 内核本来就带一个自由运行的周期计数器 DWT->CYCCNT，
 * 直接读寄存器即可，零额外外设、零中断。
 *
 * 三行的含义：
 *   DEMCR |= TRCENA      —— DWT 的总开关（不打开，DWT 的所有寄存器都读不出东西）
 *   CYCCNT = 0           —— 清零计数器，给调试时一个干净起点
 *   CTRL |= CYCCNTENA    —— 启动计数，之后每个内核时钟周期 +1
 *
 * 它数的是"CPU 周期"，不是"时间"，所以换算微秒要除以 (主频/1e6)，见 DHT11_GetUs。 */
static void DHT11_DWT_Enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* 允许访问 DWT */
    DWT->CYCCNT  = 0;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;          /* 开启周期计数 */
}

/* 返回自开机以来的微秒数。
 *
 * 换算：(SystemCoreClock / 1000000) 就是"一个微秒等于多少个 CPU 周期"。
 *       72MHz → 除数 72；48MHz → 48。DHT11_Init 必须在 SystemClock_Config() 之后
 *       调用，原因就在这里：这个除数在编译期不固定，是运行时读的变量。
 *
 * 为什么用 static 局部变量做懒初始化（而不是只在 Init 里调 DHT11_DWT_Enable）：
 *   这是一层防御——万一有人在 Init 之前就调了读取函数，DWT 没使能的话 CYCCNT
 *   恒为 0，所有超时都会"立刻超时"、所有位宽都是 0，现象非常迷惑（表现为
 *   读回 0.0 但 ok 计数在涨）。懒初始化让这个函数自己就是自洽的，谁先调都不会错。
 *   代价是每次调用多一次 if 判断，相对 4ms 的读取时间可以忽略。
 *
 * 溢出问题（面试常问）：
 *   CYCCNT 是 32 位，72MHz 下 2^32/72e6 ≈ 59.6 秒就翻回 0。
 *   但本文件所有计时代码都用"当前值 - 起始值"的 uint32 无符号减法，
 *   模 2^32 运算下即使中间跨越翻转点，差值依然正确，所以翻转不需要特殊处理。
 *   ——这就是"不要用有符号数存 tick、也不要写成 a < b 比大小"的原因。
 *
 * 一次除法约几十个周期，单帧要算上百次，仍然在 4ms 预算内，不必为省这点去做移位优化。 */
static uint32_t DHT11_GetUs(void)
{
    static uint8_t dwt_inited = 0;
    if (!dwt_inited) {
        DHT11_DWT_Enable();
        dwt_inited = 1;
    }
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

/* 等待采集线上电平达到 target，超过 timeoutUs 未到则返回 0（超时失败）。
 *
 * 为什么必须有超时，不能 while(电平不对) 死等：
 *   传感器没插、线断了、或者上拉缺失导致线一直为低——死等会把整个主循环永久卡住，
 *   现象是"程序跑飞但没进 HardFault"，极难定位。带超时的等待是驱动的基本素养：
 *   任何对外部硬件的等待都必须有出口。
 *
 * timeoutUs 取 200us 的依据（= DHT11_LEVEL_TIMEOUT_US）：
 *   本层要等的几种电平最长是"应答 80us"和"bit 起始低电平 50us"，
 *   200us 约为最大值的 2.5 倍，既容得下正常的时序抖动，又能在真的没响应时快速退出。
 *   定得太小会在不同批次的传感器上偶发失败，定得太大则单帧最坏耗时变长。 */
static uint8_t DHT11_WaitLevel(GPIO_PinState target, uint32_t timeoutUs)
{
    uint32_t start = DHT11_GetUs();
    while (DHT11_Read() != target) {
        if (DHT11_GetUs() - start > timeoutUs) { return 0; }
    }
    return 1;
}

/* 测量采集线"正在保持高电平"的时间（us），超时按最大值返回。
 *
 * 这是判 0/1 的核心：DHT11 每位都以 50us 低电平开头，后面接的高电平宽度决定数值
 *   —— 高电平 26~28us  →  逻辑 0
 *   —— 高电平 约 70us  →  逻辑 1
 * 所以"量宽度"这一件事就能解码。
 *
 * 超时为什么返回 timeoutUs（而不是返回 0 或报错）：
 *   超时说明线被一直钳在高电平（上拉把它托住了、传感器松了）。这种情况下把它当成
 *   一个"超级宽的 1"，让本帧的校验和去失败，比在计时层抛错误更省代码路径；
 *   而且校验和本来就能兜住这种错误。
 *   （反过来想：如果这里返回 0，就会静默地把一整帧错误数据当成合法的 0 交上去。） */
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
/* 调试观测区：只在 DHT11_DEBUG_TIMES=1 时编译进去，占 RAM 约 80+ 字节，
 * 关掉时一行代码都不生成，不影响时序。用法见 dht11.h 里该宏的注释。 */
uint16_t DHT11_DbgBitUs[40];   /* 40 位各自的高电平宽度（us） */
uint8_t  DHT11_DbgRaw[5];      /* 最近一次读到的原始 5 字节 */
uint8_t  DHT11_DbgFailStage;   /* 最近一次失败的阶段，见下方 STAGE_* 定义 */
uint8_t  DHT11_DbgFailBit;     /* 失败发生在第几位（仅 bit 阶段有效） */

/* 失败阶段编号：定位「到底卡在哪一步」。
 * 为什么要分这么细：DHT11 出问题时现象都是"读失败"，但原因完全不同——
 *   没应答（接线/供电）、应答被拉长（上拉缺失）、帧中途断（干扰/关中断不够）、
 *   校验错（时序勉强读全但位数错了）。分开记，一次示波器/日志就能定性。 */
#define STAGE_NONE        0    /* 未失败 */
#define STAGE_RESP_LOW    1    /* 释放总线后等不到 DHT11 拉低（从机没应答） */
#define STAGE_RESP_HIGH   2    /* 等不到应答低电平结束（应答被拉长或线被钳低） */
#define STAGE_BIT_HIGH    3    /* 40 位中某一位等不到上升沿（帧中途断了） */
#define STAGE_CHECKSUM    4    /* 40 位读全了，但校验和不匹配 */
#define STAGE_RESP_END    5    /* 应答高电平一直不结束（线被钳高或应答异常） */

#define DHT11_FAIL_STAGE(n)   (DHT11_DbgFailStage = (n))
#else
/* 关掉调试时，把记录动作编译成空语句：
 * 上层代码里那些 DHT11_FAIL_STAGE(...) 调用点就不必加 #if 包起来，
 * 既保持源码可读，又保证不产生任何代码。 */
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
        /* 移位累位：DHT11 是高位先出（MSB first），每来一位先把已有内容左移一格、
         * 再在最低位补 0/1，移满 8 次正好凑成一个字节，比按位或 0x80>>k 更直观。
         * 字节下标 i/8、位序由移位自然给出，不需要额外的位计数变量。 */
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

    /* 3. 校验：前 4 字节和（取低 8 位）应等于第 5 字节。
     * sum 声明成 uint8_t，四个字节相加溢出时自动截断——正好就是"取低 8 位"，
     * 不需要手动 & 0xFF。这也是为什么校验和能在只多一个字节的情况下抓到
     * 绝大多数时序错位/干扰错误：任一位判错，和的低 8 位几乎必然变化。 */
    {
        uint8_t sum = (uint8_t)(bitBuf[0] + bitBuf[1] + bitBuf[2] + bitBuf[3]);
        if (sum != bitBuf[4])
        {
            DHT11_FAIL_STAGE(STAGE_CHECKSUM);
            return DHT11_ERROR;
        }
    }

    /* 4. 保存结果。
     * 注意写入时机：全部校验通过之后才写 dht —— 这样失败时句柄里保留的是上一次
     * 的成功数据，上层"出错就显示旧值 + 错误计数"的逻辑才成立。
     * 温度用 int8_t 强转：DHT11 本身只报 0~50℃，但同一份代码接 DHT22 时会出现负温，
     * 提前按有符号存，换传感器不用改这里。 */
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
 *   若本函数被外层临界区包裹，直接使能会提前打开中断、破坏外层保护。
 *   __get_PRIMASK() 取到的是"调用前的真实状态"，退出时按原样写回，
 *   无论外层是开中断还是关中断，本函数都是"透明"的——这是可重入临界区的标准写法，
 *   和自己实现的嵌套计数（nesting counter）是同一目的。
 *
 * 拆成 Inner/外层两个函数，是为了让"关中断"这个横切关注点只出现在一个地方，
 * 位级逻辑里一行 __disable_irq 都没有，读起来只有时序、没有副作用。 */
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

/* 状态常量（与头文件注释对应）
 *
 * 为什么只有两个状态、为什么不用 HAL_Delay(20) 了事：
 *   起始信号要求"拉低 ≥18ms"，如果写成"拉低 → HAL_Delay(20) → 释放"，
 *   这 20ms 主循环什么都干不了。改成状态机后，拉低的动作发出就返回，
 *   20ms 的等待交给下一次调用去判断——主循环照常跑 Modbus 收帧，只是
 *   期间多了一句"现在还在等吗"的判断。
 *   实际上"拉低"这个动作本身就足以让线保持低电平，CPU 不需要守着它。 */
#define DHT11_ST_WAIT        0   /* 等待读取间隔（2s） */
#define DHT11_ST_START_LOW   1   /* 起始信号：拉低中（20ms） */

/* 初始化：只管"引脚方向 + 计时器 + 状态机起点"三件事。
 *
 * 顺序有讲究：先 DHT11_GetUs() 触发 DWT 使能，再使能端口时钟、配引脚，
 * 这样即使后面业务代码立刻调用 Handler，微秒计时也已经是可用状态。
 *
 * 引脚初始化为输入上拉 = 总线空闲态：主机不占线，随时等传感器说话。
 *
 * 留意一点（不改，仅提示）：句柄的结果字段（humidity_int/temp_int 等）没有清零，
 *   如果句柄是全局变量，首次成功读取前 DHT11_GetTemp() 返回的是 0.0；
 *   若句柄是栈上的局部变量，返回的就是随机值。上层若要在"第一次读成功之前"
 *   就显示数据，需要自己判断是否已经收到过 DHT11_OK。 */
void DHT11_Init(DHT11_Handle_t *dht)
{
    DHT11_GetUs();              /* 首次调用会顺带使能 DWT */
    DHT11_ClockEnable();
    DHT11_SetInput();           /* 初始处于输入上拉（总线释放态） */
    dht->state     = DHT11_ST_WAIT;
    dht->last_tick = HAL_GetTick();
}

/* 非阻塞处理函数：主循环反复调用，内部两态循环。
 *
 * 时序图（一轮完整周期）：
 *   ┌─ 等 2000ms ─┐┌ 拉低 20ms ┐┌─ 释放总线 + 阻塞读 4ms ─┐
 *   状态 WAIT      → 状态 START_LOW → 立即读帧 → 回 WAIT（重新计时）
 *
 * 注意 last_tick 在"进入拉低"和"读完之后"各刷新一次，所以真实周期 =
 *   2000ms + 20ms + 约4ms ≈ 2.024s，比手册要求的 ≥1s 宽裕得多，无需凑整。
 *
 * 为什么释放总线后必须"立刻"进 DHT11_ReadFrame：
 *   DHT11 收到起始信号后约 20~40us 就开始拉低应答，中间没有任何缓冲，
 *   晚一步就可能错过应答的第一个下降沿（甚至错过整段应答）。
 *   所以从 SetInput() 到 ReadFrame() 之间不允许插入任何耗时操作，
 *   这也是这里没有先 return 而是直接 return DHT11_ReadFrame(dht) 的原因——
 *   把"读"当成同一拍里的动作，而不是下一拍的事。
 *
 * 返回 DHT11_BUSY 的那两次调用不改变任何数据，上层可以放心每轮都调。 */
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

    default:                                  /* 状态变量被意外改坏（栈溢出/野指针）*/
        return DHT11_ERROR;
    }
}

/* ========================================================================== */
/* 结果读取                                                                   */
/* ========================================================================== */

/* 把"整数 + 小数"两字段拼成浮点，避免上层自己 dht->temp_int + 0.1*dec 时写漏。
 *
 * 除以 10.0f 而不是写成 (float)temp_dec / 10：DHT11 的小数位恒定上报 0，
 * 这条除法实际不起作用；但同一份代码接 DHT22 时小数位是有效的十分位，/10 才正确。
 * 也就是说这不只是为了 DHT11 写的，接 DHT22 时改掉 DHT11_BIT_1_US 阈值、
 * 关掉 20ms 起始信号的差异即可复用。
 *
 * 另外注意返回的是"物理量"（59.0），不是"×10 的整数"（590）——
 * 要送进 Modbus 寄存器（16 位整型）时，上层需要自己乘 10 取整，这一步刻意留在业务层，
 * 因为"寄存器放什么单位"是应用协议的事，不该由传感器驱动决定。 */
float DHT11_GetTemp(DHT11_Handle_t *dht)
{
    return (float)dht->temp_int + (float)dht->temp_dec / 10.0f;
}

float DHT11_GetHumi(DHT11_Handle_t *dht)
{
    return (float)dht->humidity_int + (float)dht->humidity_dec / 10.0f;
}
