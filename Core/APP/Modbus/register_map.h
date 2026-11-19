#ifndef __REGISTER_MAP_H__
#define __REGISTER_MAP_H__

#include <stdint.h>
#include <stdbool.h>

/*
 * register_map.h —— 寄存器表（Modbus 数据模型层）
 *
 * 【本层职责】
 *   只管"有哪些寄存器、当前值是多少、谁能读写"，不认识帧格式、
 *   不认识功能码、不认识串口。协议层通过 reg_read/reg_write 取数，
 *   应用层（采样、显示）也通过同一套接口，双方都不直接碰存储数组。
 *
 * 【为什么要有这一层】
 *   重构前 main.c 直接写 ModbusRegister[0]，等于业务层越过协议层
 *   直接改数据模型。加一个传感器就要动 main.c，且没有任何越界保护。
 *   收进本层后，越界检查只有一处，所有读写路径都必须经过它。
 *
 * 【当前状态】
 *   - 保持寄存器 2 个（温度、湿度），供 0x03 读取
 *   - 线圈表保留但未启用（0x01/0x05/0x0F 功能码目前为注释状态）
 *   - 所有访问都在主循环上下文，不进中断，因此不需要临界区保护
 */

/* 保持寄存器数量 —— 唯一权威定义。
 * 重构前 modbus_rtu.h 里写的是 NUMBER_OF_REGISTER=10，而实际数组只有 2 个元素，
 * 一旦启用 0x06/0x10 就会按 10 做越界检查通过、再写坏相邻内存（B5）。
 * 现在数组长度和数量宏同源，不可能再对不上。 */
#define REG_COUNT   2

/* 线圈数量（功能码未启用，仅保留类型定义，加 TODO 便于以后恢复） */
#define COIL_COUNT  30

/* 寄存器元数据：给显示层 / 上位机用的描述信息，与协议无关。
 * 有了它，OLED 或调试页可以遍历打印，不用在显示代码里硬编码"第 0 个是温度"。 */
typedef struct
{
    const char *name;    /* 名称，如 "温度" */
    const char *unit;    /* 单位，如 "℃" */
    uint8_t     scale;   /* 放大倍数：存储值 250 表示 25.0 → scale = 10 */
} reg_meta_t;

/* 初始化：把所有寄存器清零。
 * 必须在 main 的初始化阶段调用一次，避免上电后是随机值。 */
void reg_init(void);

/* 读一个寄存器。
 *   越界返回 0（读不该破坏任何状态，所以静默返回 0 而不是报错）。
 *   注意：返回 0 无法区分"越界"和"值就是 0"，需要区分时先用 reg_count() 判。 */
uint16_t reg_read(uint16_t addr);

/* 写一个寄存器。
 *   越界返回 false 且不写入 —— 这是最后一道防线（B6）：
 *   协议层虽然已按 reg_count() 检查过，但应用层也可能传错地址，
 *   写坏相邻内存属于最难查的故障，宁可多拦一次。 */
bool reg_write(uint16_t addr, uint16_t value);

/* 寄存器总数（协议层做 0x03 的地址范围检查时用这个，不要用宏） */
uint16_t reg_count(void);

/* 取元数据；越界返回 NULL。 */
const reg_meta_t *reg_meta(uint16_t addr);

/* ================== 线圈（预留，功能码未启用） ================== */
/* TODO: 0x01 / 0x05 / 0x0F 功能码在 modbus_proto.c 中是注释状态，
 *       恢复它们时用下面两个接口，不要再去直接访问存储数组。 */
bool coil_read(uint16_t addr);
bool coil_write(uint16_t addr, bool value);
uint16_t coil_count(void);

#endif /* __REGISTER_MAP_H__ */
