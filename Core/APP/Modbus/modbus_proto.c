/*
 * modbus_proto.c —— Modbus RTU 协议层实现
 *
 * 只做两件事：校验进来的帧、组出去发的帧。
 * 不发送、不路由、不计数（原因见 modbus_proto.h）。
 */
#include "modbus_proto.h"
#include "register_map.h"
#include "CRC16.h"

/* Modbus 单帧最小长度：地址 + 功能码 + CRC(2) = 4 */
#define MB_MIN_FRAME_LEN    4

/* 0x03 单次最多可读的寄存器数（规范上限 125） */
#define MB_03_MAX_REG       125

/* ================== 帧校验 ================== */

mb_frame_check_t mb_proto_check(const uint8_t *frame, uint8_t len)
{
    if (len < MB_MIN_FRAME_LEN)
    {
        return MB_FRAME_TOO_SHORT;  //帧太短
    }

    /* modbus_crc16 的返回值语义：对"含 CRC 的完整帧"再算一遍，
     * 结果为 0 表示正确，非 0 表示帧被破坏。 */
    if (modbus_crc16((const char *)frame, len) != 0)
    {
        return MB_FRAME_CRC_ERR;  //CRC校验错误
    }

    return MB_FRAME_OK;
}

/* ================== 异常帧 ================== */

uint8_t mb_proto_exception(const uint8_t *req, uint8_t code, uint8_t *out, uint8_t out_cap)
{
    uint16_t crc;

    if (out_cap < 5)
    {
        return 0;
    }

    /* 先读 req 再写 out：这样 out 与 req 指向同一缓冲也安全。 */
    out[0] = req[0];
    out[1] = (uint8_t)(req[1] | 0x80);
    out[2] = code;

    crc = modbus_crc16((const char *)out, 3);
    out[3] = (uint8_t)(crc & 0x00FF);     /* CRC 低字节在前 */
    out[4] = (uint8_t)(crc >> 8);

    return 5;
}

/* ================== 0x03 读保持寄存器 ================== */

uint8_t mb_proto_build_03(const uint8_t *req, uint8_t len, uint8_t *out, uint8_t out_cap)
{
    uint16_t addr;      //起始地址
    uint16_t reg_num;   //读取寄存器数量
    uint16_t i;
    uint16_t need;
    uint16_t crc;

    (void)len;   /* 长度已在 mb_proto_build 的统一入口检查过 */

    /* 大端：高字节在前 */
    addr    = (uint16_t)(((uint16_t)req[2] << 8) | req[3]);
    reg_num = (uint16_t)(((uint16_t)req[4] << 8) | req[5]);

    if (reg_num == 0 || reg_num > MB_03_MAX_REG)
    {
        return mb_proto_exception(req, MB_EX_ILLEGAL_VALUE, out, out_cap);
    }

    /* 地址范围检查用 reg_count()，不用宏 —— 寄存器数量和数组长度同源，
     * 不会像重构前那样出现"检查按 10 通过、实际数组只有 2"的越界（B5）。 */
    if ((uint32_t)addr + reg_num > reg_count())
    {
        return mb_proto_exception(req, MB_EX_ILLEGAL_ADDRESS, out, out_cap);
    }

    /* 应答总长 = 地址1 + 功能码1 + 字节数1 + 数据(2×N) + CRC2 */
    need = (uint16_t)(5 + reg_num * 2);

    if (need > out_cap)
    {
        return 0;    /* 容量不足，调用方不应发送 */
    }

    out[0] = req[0];
    out[1] = req[1];
    out[2] = (uint8_t)(reg_num * 2);     /* 字节数 */

    for (i = 0; i < reg_num; i++)
    {
        uint16_t value = reg_read((uint16_t)(addr + i));

        /* 16 位寄存器拆成高字节在前 */
        out[3 + i * 2] = (uint8_t)(value >> 8);
        out[4 + i * 2] = (uint8_t)(value & 0x00FF);
    }

    crc = modbus_crc16((const char *)out, (uint8_t)(3 + reg_num * 2));
    out[3 + reg_num * 2] = (uint8_t)(crc & 0x00FF);
    out[4 + reg_num * 2] = (uint8_t)(crc >> 8);

    return (uint8_t)need;
}

/* ================== 统一组包入口 ================== */

uint8_t mb_proto_build(const uint8_t *req, uint8_t len, uint8_t *out, uint8_t out_cap)
{
    /* 最小帧长：Modbus 标准请求都是 8 字节起
     * （地址 + 功能码 + 起始地址2 + 数量2 + CRC2）。
     * 下面的 build_03 要读 req[2]~req[5]，长度不够会读到上一条帧的残留字节，
     * 组出一个地址/数量全是垃圾的应答。所以统一在这里挡掉。
     * 此时 req[0]/req[1] 必然有效（mb_proto_check 已保证至少 4 字节）。 */
    if (len < 8)
    {
        return mb_proto_exception(req, MB_EX_ILLEGAL_VALUE, out, out_cap);
    }

    switch (req[1])
    {
    case MB_FC_READ_HOLDING:
        return mb_proto_build_03(req, len, out, out_cap);

    /* TODO: 以下功能码为预留状态，恢复时按下表实现，接口保持不变。
     *       注意 0x06/0x10 的写操作必须走 reg_write()，不能直接访问存储数组。 */
    // case MB_FC_READ_COIL:         return mb_proto_build_01(...);
    // case MB_FC_WRITE_SINGLE_COIL: return mb_proto_build_05(...);
    // case MB_FC_WRITE_SINGLE_REG:  return mb_proto_build_06(...);
    // case MB_FC_WRITE_MULTI_COIL:  return mb_proto_build_15(...);
    // case MB_FC_WRITE_MULTI_REG:   return mb_proto_build_16(...);

    default:
        /* 不支持的功能码 → 异常码 0x01，功能码最高位置 1 */
        return mb_proto_exception(req, MB_EX_ILLEGAL_FUNCTION, out, out_cap);
    }
}

/* ================== 工具函数 ================== */

uint8_t mb_proto_byte_count(uint16_t bits)
{
    if (bits == 0)
    {
        return 0;
    }

    return (uint8_t)((bits + 7) / 8);
}
