/*
 * modbus_rtu.c —— Modbus 路由与收发出口（薄壳）
 *
 * 原始来源 : Modbus_Lib.c  (Ümit Can Güveren, 2022)
 *
 * 【重构历史】
 *   v1  单文件 385 行，同时承担协议实现、地址路由、诊断计数、出口决策。
 *   M2  帧接收与判帧拆到 Transport/frame.c。
 *   M3  本文件拆成三块：register_map（数据模型）、modbus_proto（协议）、
 *       App/diag（诊断），本文件只留"取帧 → 校验 → 路由 → 发送"。
 *
 * 【关键设计：协议层不发送】
 *   modbus_proto 的所有 build 函数只往缓冲写字节并返回长度，
 *   出口选择在这里做。这样 M4 插入 router 层时，改动只落在本文件的
 *   switch 上，协议层一行不动。
 */
#include "modbus_rtu.h"
#include "frame.h"
#include "modbus_proto.h"
#include "register_map.h"
#include "diag.h"
#include "Mb_port.h"
#include <stddef.h>   /* NULL */

/* ================== 从站地址 ================== */

/* 0x01：RS485 总线上的外部温湿度变送器 —— 本机只原样透传，由它自己应答。
 * 0x02：本机从站 —— 用 register_map 里的数据组包应答。 */
#define SLAVE_ADDR_REMOTE   0x01
#define SLAVE_ADDR_LOCAL    0x02

/* ================== 收发缓冲 ================== */

/* 为什么要"拷出来"：frame_poll 会把 frame 层的缓冲清零并复用，
 * 而发送是阻塞的、耗时较长，这段时间里新帧可能已经开始写入。
 * 所以取帧时直接拷到本模块自己的缓冲，发送期间不受中断影响。 */
static uint8_t s_pc_frame[FRAME_BUF_SIZE];       //PC端数据缓存区
static uint8_t s_rs485_frame[FRAME_BUF_SIZE];    //RS485数据缓存区
static uint8_t s_response_frame[FRAME_BUF_SIZE];      //应答帧数据组包区

/* ================== 出口决策 ================== */

/* 应答方向规则：
 *   0x01 的应答 → RS485（由总线上的变送器发出，本机不产生）
 *   0x02 的应答 → PC
 *
 * 注意这条规则只对"应答方向"成立。主站发送时发给 01/02 的请求
 * 都走同一条 485，地址不参与出口决策 —— 主站自己指定端口。
 *
 * TODO(M4): 本函数与 modbus_poll_pc_frames 里的地址 switch 一起搬到 App/router.c。 */
static mb_port_t reply_port(uint8_t addr)
{
    return (addr == SLAVE_ADDR_REMOTE) ? MB_PORT_RS485 : MB_PORT_PC;
}

static void send_by_addr(uint8_t addr, const uint8_t *buf, uint8_t len)
{
    (void)mb_port_send(reply_port(addr), buf, len);
}

/* ================== 初始化 ================== */

void modbus_init(void)
{
    reg_init();     /* 寄存器清零：上电后不能是随机值 */
    diag_reset();   /* 诊断计数清零 */
}

/* ================== PC 侧消费者 ================== */

void modbus_poll_pc_frames(void)
{
    uint8_t overflowed = 0;
    uint8_t len;
    uint8_t resp_len;

    len = frame_poll(FRAME_CH_PC, s_pc_frame, &overflowed);      //取PC端数据   pc端数据缓存区  超长标志存储地址

    if (len == 0)
    {
        return;     /* 没有整帧，正常的空闲状态 */
    }

    /* 超长必须在 CRC 之前判：超长帧的尾部字节已经被 frame 层丢弃，
     * 剩下的部分算 CRC 必然失败，报出来就成了"CRC 错"，掩盖真实原因。 */
    if (overflowed)
    {
        diag_inc(DIAG_FRAME_LONG);      //LONG自增1次
        return;
    }

    switch (mb_proto_check(s_pc_frame, len))     //帧校验,三个返回值
    {
    case MB_FRAME_OK:               //帧完整
        break;

    case MB_FRAME_TOO_SHORT:        //帧过短
        diag_inc(DIAG_FRAME_SHORT);
        return;

    case MB_FRAME_CRC_ERR:          //帧CRC错误
        diag_inc(DIAG_CRC_ERR);
        return;

    default:
        return;
    }

    /* ---- 地址路由 ---- */
    switch (s_pc_frame[0])
    {
    case SLAVE_ADDR_REMOTE:
        /* 透传：不改内容、不做功能码处理，由变送器自己应答。
         * 它的应答经 USART2 回来，由 modbus_poll_rs485_frames 回显给 PC。 */
        send_by_addr(SLAVE_ADDR_REMOTE, s_pc_frame, len);
        break;

    case SLAVE_ADDR_LOCAL:
        /* 本机应答：协议层组包（不发送），这里决定往哪发。
         * 不支持的功能码、长度不足、地址越界，都由 mb_proto_build
         * 内部转成异常帧返回，不需要在这里再判一次。 */
        resp_len = mb_proto_build(s_pc_frame, len, s_response_frame, (uint8_t)sizeof(s_response_frame));

        if (resp_len > 0)
        {
            send_by_addr(SLAVE_ADDR_LOCAL, s_response_frame, resp_len);
        }
        break;

    default:
        /* 既非 01 也非 02：不是给本机的帧，静默丢弃并计数。 */
        diag_inc(DIAG_BAD_ADDR);    //非法地址计数器加1
        break;
    }
}

/* ================== RS485 侧消费者 ================== */

void modbus_poll_rs485_frames(void)
{
    uint8_t len = frame_poll(FRAME_CH_RS485, s_rs485_frame, NULL);

    if (len == 0)
    {
        return;
    }

    /* 原样回显给 PC。出口固定是 PC，不走 reply_port ——
     * 因为这是"把总线上的应答转给上位机"，不是"本机应答"。 */
    (void)mb_port_send(MB_PORT_PC, s_rs485_frame, len);
}
