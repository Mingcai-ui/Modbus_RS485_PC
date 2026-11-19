#ifndef __MODBUS_PROTO_H__
#define __MODBUS_PROTO_H__

#include <stdint.h>

/*
 * modbus_proto.h —— Modbus RTU 协议层（纯协议，不做 I/O）
 *
 * 【本层职责】
 *   帧校验 + 组应答帧。输入是"请求帧字节"，输出是"应答帧字节"。
 *
 * 【本层不认识什么】（这是分层的核心）
 *   - 不认识串口 / 端口 / RS485 方向脚  → 由调用方发送
 *   - 不认识从站地址路由规则            → 由 modbus_rtu.c 的路由决定
 *   - 不认识诊断计数                    → 由 App 层决定怎么记
 *
 * 【为什么组包函数不自己发送】
 *   重构前 makePacket_03() 组完包直接 sendMessage()，出口规则（按地址
 *   选 PC 还是 RS485）就焊死在协议层里。M4 要插一个 router 层做地址
 *   路由，如果协议层自己发送，router 就没有插入点，到时还得再拆一次。
 *
 *   所以本层所有 build_* 函数都是"写入 out 缓冲并返回长度"，
 *   返 0 表示不该回应。发送是调用方的事。
 *
 * 【缓冲约定】
 *   所有帧缓冲统一用 uint8_t（不用 char）：
 *   原代码满屏 (uint8_t)msg[3] 强转，根因是 char 在 ARM 上默认为有符号。
 *   用 uint8_t 后不再依赖 -funsigned-char 这个编译开关。
 */

/* ================== 功能码 ================== */
enum
{
    MB_FC_READ_COIL          = 0x01,
    MB_FC_READ_HOLDING       = 0x03,
    MB_FC_WRITE_SINGLE_COIL  = 0x05,
    MB_FC_WRITE_SINGLE_REG   = 0x06,
    MB_FC_WRITE_MULTI_COIL   = 0x0F,
    MB_FC_WRITE_MULTI_REG    = 0x10,
};

/* ================== 异常码 ================== */
#define MB_EX_ILLEGAL_FUNCTION  0x01    /* 从机不支持该功能码 */
#define MB_EX_ILLEGAL_ADDRESS   0x02    /* 数据地址越界 */
#define MB_EX_ILLEGAL_VALUE     0x03    /* 数据值非法 */

/* 帧校验结果。
 * 注意：这里没有"太长"——超长证据只存在于中断里（frame 层的 overflow），
 * 由调用方先判断，再进本层做 CRC 校验。 */
typedef enum
{
    MB_FRAME_OK = 0,
    MB_FRAME_TOO_SHORT,     /* len < 4：连"地址+功能码+CRC"都不够 */
    MB_FRAME_CRC_ERR,       /* CRC 不匹配 */
} mb_frame_check_t;

/* 帧校验：长度下限 + CRC。
 * CRC 的判定方式是"对整个帧（含末尾 2 字节 CRC）再算一遍，结果为 0 即正确"，
 * 这是 Modbus CRC 的数学性质，不是笔误。 */
mb_frame_check_t Modbus_proto_check(const uint8_t *frame, uint8_t len);

/* 组应答帧（统一入口）。
 *   req      : 请求帧（只读）
 *   len      : 请求帧长度
 *   out      : 输出缓冲（至少 out_cap 字节）
 *   out_cap  : 输出缓冲容量
 *   返回     : 应答帧长度；0 = 无法组包（容量不足），调用方不应发送
 *
 * 内部按功能码分发；不支持的码返回异常帧（而不是 0）——
 * 因为"不支持"本身是需要告知主站的合法应答。
 *
 * ★ req 与 out 可以是同一块缓冲：本函数在写 out 之前已把需要的信息
 *   全部读入局部变量，之后再也不会读 req。
 */
uint8_t mb_proto_build(const uint8_t *req, uint8_t len, uint8_t *out, uint8_t out_cap);

/* 组异常应答帧：地址 + (功能码|0x80) + 异常码 + CRC = 5 字节。
 * 返回长度；容量不足返回 0。 */
uint8_t mb_proto_exception(const uint8_t *req, uint8_t code, uint8_t *out, uint8_t out_cap);

/* 0x03 读保持寄存器。
 * 数据来源是 register_map（通过 reg_read），本函数不直接接触存储数组。 */
uint8_t mb_proto_build_03(const uint8_t *req, uint8_t len, uint8_t *out, uint8_t out_cap);

/* 计算 N 个位（线圈）需要几个数据字节：(N+7)/8。
 * TODO: 只被 0x01 使用，而 0x01 目前是注释状态。恢复 0x01 时保留此函数。 */
uint8_t mb_proto_byte_count(uint16_t bits);

#endif /* __MODBUS_PROTO_H__ */
