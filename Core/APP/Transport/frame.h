#ifndef __FRAME_H__
#define __FRAME_H__

#include <stdint.h>

/*
 * frame.h —— 帧收发服务层（Transport 层）对外接口
 *
 * 【这一层解决的问题】
 *   "串口上来的是一串字节，怎么知道哪里是一帧的开始和结束？"
 *   这件事和 Modbus 无关（CAN、自定义协议都这么做），所以单独成层：
 *   本层只负责把字节流切成帧，不理解帧里是什么内容。
 *
 * 【谁调用我】
 *   上层（modbus_rtu.c 的消费者）只在主循环里调 frame_poll() 取帧；
 *   中断回调由 HAL 直接调本层的 HAL_UART_RxCpltCallback()，不经过上层。
 *
 * 【我依赖谁】
 *   只依赖 usart.h 里的 huart1 / huart2 和 HAL 的接收 API。
 *   ★ 禁止在本层 include modbus_rtu.h 或任何协议头文件，
 *     否则又会形成"服务层反向依赖协议层"的循环。
 *
 * 【缓冲大小】
 *   255 = Modbus RTU 单帧合法上限，不可升到 256：
 *   rx_count 是 uint8_t，计到 256 会回绕成 0，导致帧头被尾部数据覆盖。
 */
#define FRAME_BUF_SIZE 255

typedef enum
{
    FRAME_CH_PC = 0,     /* USART1：PC 上位机（二进制 Modbus 帧） */
    FRAME_CH_RS485,      /* USART2：RS485 总线（外部变送器应答） */
    FRAME_CH_COUNT       /* 通道总数，兼作 channels[] 数组长度 */
} frame_ch_t;

/* 初始化：启动两路中断接收。必须在 MX_USARTx_UART_Init() 之后调用一次。 */
void frame_init(void);

/* 中断回调实体：收到一个字节。
 * 正常情况下由本文件的 HAL_UART_RxCpltCallback() 调用，
 * 头文件里暴露出来是为了可测试 / 可被其它运输方式（如 DMA）复用。 */
void frame_on_byte(frame_ch_t ch, uint8_t byte);

/* 中断回调实体：接收出错（ORE 等），丢弃当前帧并重新武装接收。 */
void frame_on_error(frame_ch_t ch);

/* 1ms 判帧节拍（须注册到 tick：tick_register(frame_tick)）。
 * 内部遍历所有通道，累加静默时间，够 5ms 就置位 frame_ready。 */
void frame_tick(void);

/* 取一帧（主循环上下文，唯一的数据出口）。
 *   返回  : 帧长度，0 = 当前没有就绪帧（不是错误，直接 return 即可）
 *   out   : 调用者提供的缓冲，长度至少 FRAME_BUF_SIZE
 *   溢出  : *overflowed 由本函数写出（取走即清零）；传 NULL 表示不关心。
 *           为 1 时表示该帧在接收期间曾超出缓冲，尾部字节已丢，
 *           上层应判为 FRAME_Too_LONG 而不是拿去校验 CRC。
 * 注意：返回值是"本次拷出的字节数"，与 Modbus 帧头里声明的长度无关。 */
uint8_t frame_poll(frame_ch_t ch, uint8_t *out, uint8_t *overflowed);

#endif /* __FRAME_H__ */
