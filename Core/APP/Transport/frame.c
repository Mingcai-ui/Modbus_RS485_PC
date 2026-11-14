/*
 * frame.c —— 帧收发服务层 (Transport) 实现
 *
 * 【本层职责】
 *   把"串口字节流"变成"完整帧"这件与协议无关的事，单独封装起来：
 *     - 接收缓冲管理（收字节、防溢出）
 *     - 帧边界判定（5ms 静默法）
 *     - 中断入口（HAL 回调只和本层对接）
 *
 * 【依赖方向】上层(协议/路由/应用) → 本层 → HAL
 *   本层不认识 Modbus、不认识功能码、不认识寄存器。
 *   上层通过 frame_poll() 取帧，不得直接访问 channels[]。
 *
 * 【执行上下文】本层代码在三种上下文中运行，靠 volatile + 共享帧槽通信：
 *   - frame_on_byte()  : USART 接收中断（每字节一次）
 *   - frame_tick()     : SysTick 中断（每 1ms 一次）
 *   - frame_poll()     : 主循环 while(1)
 *   三者之间没有锁，靠"中断只写、主循环只取"的单向数据流保证安全。
 */
#include "frame.h"
#include "usart.h"
#include "string.h"

/* 帧间静默判帧时间 (ms)
 * 9600bps 8N1 : 1 字符 = 10bit / 9600 = 1.04ms , 3.5T ≈ 3.65ms
 * 取 5ms 留出 RS485 收发器切换余量；SysTick 1ms 计数，精度 1ms 够用
 *
 * 为什么不用 UART IDLE 中断（更精确）？
 *   1. STM32F1 的 IDLE 中断通常配合 DMA 使用，本工程是单字节 IT；
 *   2. IDLE 检测的是"线路空闲"，RS485 半双工在 DE 切换瞬间也可能
 *      产生空闲电平抖动，反而误判;
 *   3. 5ms 软定时不额外占用外设资源，两路串口共用一套逻辑。 */
#define FRAME_GAP_MS 5

/* 单个通道的帧槽 —— 把所有跨上下文共享的状态集中成一块内存
 *
 * 这个结构体取代了原来的 6 个散落全局变量：
 *   rx_active     ← RxInterruptFlag / RxInterruptFlag_PC
 *   time_counter  ← uartTimeCounter / uartTimeCounter_PC
 *   frame_ready   ← uartPacketComplatedFlag / uartPacketComplatedFlag_PC
 * 另外新增 overflow，把"超长"这个判决依据从 USART1 单侧扩展到两路对称
 * （原 USART2 侧没有超长标志，超长时只是静默停止写入）。
 *
 * 【为什么每个字段都必须 volatile】
 *   这些字段在中断里被改写，但主循环侧的代码看不到任何"写"操作。
 *   若不声明 volatile，编译器会认为 frame_ready 在 while(1) 里永不改变，
 *   把读取优化成"只读一次内存"，导致 frame_poll 永远返回 0。
 *   这是裸机中断编程最常见、也最难定位的一类 bug。 */
typedef struct
{
    uint8_t  rx_buf[FRAME_BUF_SIZE]; /* 帧数据缓冲（本帧已收的字节） */
    volatile uint8_t rx_byte;        /* HAL 单字节接收落点（中断写，必须常驻内存） */
    volatile uint8_t rx_count;       /* 本帧已收字节数（也是下一个写入下标） */
    volatile uint8_t overflow;       /* 本帧曾超出缓冲，尾部字节已被丢弃 */
    volatile uint8_t rx_active;      /* 总线正在接收中（原 RxInterruptFlag） */
    volatile uint8_t time_counter;   /* 距上一个字节的静默毫秒数 */
    volatile uint8_t frame_ready;    /* 完整帧就绪，等待 frame_poll 取走 */
} frame_slot_t;

/* 通道表：用"结构体数组 + 枚举下标"代替"变量名加 _PC 后缀"的复制粘贴。
 * 新增第 3 路串口只需在 frame_ch_t 枚举里加一项、在 ch_huart() 里加一行映射，
 * frame_on_byte / frame_tick / frame_poll 的逻辑代码一行都不用改。 */
static frame_slot_t channels[FRAME_CH_COUNT];

/* 通道 → HAL 句柄 映射。
 * 把对 huart1/huart2 的依赖关在本文件内部：上层看不到 HAL 句柄，
 * 换串口、换芯片只改这一个函数。 */
static UART_HandleTypeDef *ch_huart(frame_ch_t ch)
{
    return (ch == FRAME_CH_PC) ? &huart1 : &huart2;
}

/* 启动两路中断接收。
 *
 * 注意接收指针指向 channels[].rx_byte（帧槽成员），不能是局部变量：
 * HAL_UART_Receive_IT 只是登记指针后立即返回，真正的数据在中断里才写入，
 * 指针必须一直有效。这也是原工程 uart1_RxData/uart2_RxData 必须是全局变量的原因。 */
void frame_init(void)
{
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&channels[FRAME_CH_PC].rx_byte, 1);
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&channels[FRAME_CH_RS485].rx_byte, 1);
}

/* 中断回调实体：收到一个字节（USART 中断上下文）
 *
 * 三件事：写缓冲、重置静默计时、重新武装接收。 */
void frame_on_byte(frame_ch_t ch, uint8_t byte)
{
    frame_slot_t *c = &channels[ch];

    /* 【防 uint8_t 回绕 —— 本函数最关键的判断】
     * 缓冲满了绝不能继续 rx_count++。因为 rx_count 是 uint8_t，
     * 255 再加 1 会回绕成 0，后续字节就从 rx_buf[0] 开始覆盖，
     * 把帧"头"冲成尾部数据 —— 结果是一个超长帧被伪装成
     * "头部合法、长度正常"的帧，甚至可能碰巧通过 CRC。
     * 所以宁可冻结计数、丢弃尾部，只留一个 overflow 标记，
     * 让上层明确判为 FRAME_Too_LONG 并计数。 */
    if (c->rx_count < FRAME_BUF_SIZE)
    {
        c->rx_buf[c->rx_count++] = byte;
    }
    else
    {
        c->overflow = 1;
    }

    c->rx_active = 1;      /* 标记总线处于接收中，frame_tick 才会开始计时 */
    c->time_counter = 0;   /* 每收到一个字节就重置帧间隔计时 */

    /* 重新武装：HAL 的 IT 接收是一次性的，处理完必须再次登记，
     * 否则收完这个字节之后就再也进不了这个通道的中断。 */
    HAL_UART_Receive_IT(ch_huart(ch), (uint8_t *)&c->rx_byte, 1);
}

/* 接收错误恢复（USART 中断上下文）
 *
 * 典型来源：ORE（溢出错误）—— 处理太慢，上一个字节还没被取走，
 * 下一个字节就到了。此时硬件可能停止接收，必须清标志并重新武装才能恢复。 */
void frame_on_error(frame_ch_t ch)
{
    frame_slot_t *c = &channels[ch];

    __HAL_UART_CLEAR_OREFLAG(ch_huart(ch));

    c->rx_count = 0;
    c->rx_active = 0;
    c->time_counter = 0;
    /* 本帧已放弃，超长证据一并作废：否则 rx_active=0 让定时器不再判帧，
     * handler 永远不跑，这个 SET 残留到下一帧，把合法帧误判成超长。
     * 这就是"状态跨帧污染"，状态机里最经典的坑之一。 */
    c->overflow = 0;

    HAL_UART_Receive_IT(ch_huart(ch), (uint8_t *)&c->rx_byte, 1);
}

/* 判帧：每 1ms 由 tick 分发器调用一次（SysTick 中断上下文）
 *
 * 逻辑与原 uartTimer() / uartTimer_PC() 完全等价，
 * 区别只是用循环遍历两路，把两份复制粘贴的代码合成一份，并天然支持任意路数。 */
void frame_tick(void)
{
    uint8_t i;
    for (i = 0; i < FRAME_CH_COUNT; i++)
    {
        frame_slot_t *c = &channels[i];

        if (c->rx_active == 0)   /* 总线没在收，不计时 */
        {
            continue;
        }

        if (++c->time_counter >= FRAME_GAP_MS)   /* 静默够久，认为一帧结束 */
        {
            c->rx_active = 0;
            c->time_counter = 0;
            c->frame_ready = 1;      /* 取到完整的一帧数据 */
        }
    }
}

/* 取一帧（主循环上下文）
 *
 * 返回帧长度，0 表示当前没有就绪的帧。
 * *overflowed 指示本帧是否曾超出缓冲（可传 NULL 表示不关心），
 * 上层据此把"超长"和"CRC 错/太短"折叠进同一个判决变量。
 *
 * 【下面三步的顺序不能换】
 *   ① 先清 frame_ready  —— "这一帧我认领了"，防止重复取出；
 *   ② 再把 rx_count 读进局部变量 len —— 后续 memcpy 必须用一个
 *      不会被中断改写的稳定长度，不能反复去读全局计数器；
 *   ③ 最后清 rx_count    —— 为下一帧腾出写指针。
 * 若先清 rx_count 再读 len，读到的就是 0。
 *
 * 【已知窄竞态（原代码同样存在，本步未扩大也未消除）】
 *   ② 与 ③ 之间若恰好有新字节到达，该字节会写到 rx_buf[len]，
 *   随后 rx_count 被清 0，于是这个字节丢失。由于帧与帧之间至少有
 *   5ms 静默，实际几乎撞不上；严格解决属于 M5 请求-应答状态机的范围。 */
uint8_t frame_poll(frame_ch_t ch, uint8_t *out, uint8_t *overflowed)
{
    frame_slot_t *c = &channels[ch];
    uint8_t len;

    if (c->frame_ready == 0)       //判断当前是否收到完整帧
    {
        return 0;
    }

    c->frame_ready = 0;          /* ① 认领 */
    len = c->rx_count;           /* ② 取长度 */
    c->rx_count = 0;             /* ③ 腾位置 */

    if (overflowed != NULL)
    {
        *overflowed = c->overflow;
    }
    /* overflow 必须"取走即清零"：留在槽里会被下一帧误读，判成超长。
     * 这也是这里设计成出参、而不让上层直接读字段的原因。 */
    c->overflow = 0;

    if (len > 0)
    {
        memcpy(out, c->rx_buf, len);
        memset(c->rx_buf, 0, FRAME_BUF_SIZE);   /* 保留原行为的冗余清零，便于调试观察缓冲内容 */
    }

    return len;
}

/* ============ HAL 接收/错误回调（弱函数，由本层接管） ============ */

/* HAL 在中断里调用这两个弱函数，缺省实现是空的。
 * 本层接管的好处：上层任何模块都不必再关心 HAL 回调，
 * 也不必知道 huart1/huart2 的 Instance 判断逻辑。
 * 分派依据是 huart->Instance 而不是通道号 —— 因为 HAL 只告诉我们句柄。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        frame_on_byte(FRAME_CH_PC, channels[FRAME_CH_PC].rx_byte);
    }
    else if (huart->Instance == USART2)
    {
        frame_on_byte(FRAME_CH_RS485, channels[FRAME_CH_RS485].rx_byte);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        frame_on_error(FRAME_CH_PC);
    }
    else if (huart->Instance == USART2)
    {
        frame_on_error(FRAME_CH_RS485);
    }
}
