/*
 * register_map.c —— 寄存器表实现
 *
 * 存储与元数据都是文件私有（static），对外只暴露 reg_read/reg_write。
 * 这样"谁能改寄存器"就只有两个出口，越界检查不会漏。
 */
#include "register_map.h"
#include <stddef.h>   /* NULL */

/* ================== 存储区 ================== */

/* 保持寄存器。地址 0 = 温度×10，地址 1 = 湿度×10（见下方元数据表）。 */
static uint16_t s_reg_values[REG_COUNT];

/* 线圈表。功能码未启用，保留存储以便恢复 0x01/0x05/0x0F 时不改结构。 */
static bool s_coil_values[COIL_COUNT];

/* ================== 元数据表 ================== */

/* 下标必须与 s_reg_values[] 的地址一一对应。
 * 这是"轻量表驱动"：协议层只认地址，显示层靠这张表知道每个地址是什么。 */
static const reg_meta_t s_reg_meta_table[REG_COUNT] =
{
    /* 地址 0 */ { "温度", "℃",   10 },
    /* 地址 1 */ { "湿度", "%RH", 10 },
};

/* ================== 实现 ================== */

/* 上电清零。
 * 为什么要显式清：C 标准保证 static 变量零初始化，但这里再做一次
 * 是为了"复位语义明确"——将来若改成 RAM 保持区或备份域，这行不能省。 */
void reg_init(void)
{
    uint16_t i;

    for (i = 0; i < REG_COUNT; i++)
    {
        s_reg_values[i] = 0;
    }

    for (i = 0; i < COIL_COUNT; i++)
    {
        s_coil_values[i] = false;
    }
}

uint16_t reg_read(uint16_t addr)    //返回指定寄存器信息
{
    if (addr >= REG_COUNT)
    {
        return 0;
    }

    return s_reg_values[addr];
}

bool reg_write(uint16_t addr, uint16_t value)
{
    /* 最后一道越界防线（B6）。
     * 注意比较用 REG_COUNT 而不是某个"总数"宏 —— 数组长度和它同源，
     * 以后加寄存器只改 #define 一处，这里自动跟上。 */
    if (addr >= REG_COUNT)
    {
        return false;
    }

    s_reg_values[addr] = value;
    return true;
}

uint16_t reg_count(void)        //返回寄存器数量
{
    return REG_COUNT;
}

const reg_meta_t *reg_meta(uint16_t addr)       //返回显示内容,放大倍数等数据
{
    if (addr >= REG_COUNT)
    {
        return NULL;
    }

    return &s_reg_meta_table[addr];
}

/* ================== 线圈（预留） ================== */

bool coil_read(uint16_t addr)
{
    if (addr >= COIL_COUNT)
    {
        return false;
    }

    return s_coil_values[addr];
}

bool coil_write(uint16_t addr, bool value)
{
    if (addr >= COIL_COUNT)
    {
        return false;
    }

    s_coil_values[addr] = value;
    return true;
}

uint16_t coil_count(void)
{
    return COIL_COUNT;
}
