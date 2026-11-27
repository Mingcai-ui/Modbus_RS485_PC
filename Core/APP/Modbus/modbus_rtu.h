#ifndef __MODBUS_RTU_H__
#define __MODBUS_RTU_H__

#include <stdint.h>

/*
 * modbus_rtu.h —— Modbus 路由与收发出口（薄壳层）
 *
 * 【本层职责】
 *   只做三件事：
 *     1) 从 frame 层取整帧；
 *     2) 按从站地址路由（0x01 透传 / 0x02 本机应答 / 其它丢弃）；
 *     3) 按地址选出口发送。
 *
 * 【本层不再包含】（M3 拆走了）
 *   - 功能码实现   → modbus_proto.c
 *   - 寄存器存储   → register_map.c
 *   - 诊断计数     → App/diag.c
 *   - 帧校验细节   → modbus_proto.c（mb_proto_check）
 *   - 缓冲与判帧   → Transport/frame.c（M2 已拆）
 *
 * 【M4 预留】下一步会把路由部分（下面 modbus_poll_pc_frames 里的 switch）
 *   搬到 App/router.c，届时本文件只剩"取帧 + 调 router + 发送"。
 */

/* 初始化：清空寄存器表、清零诊断计数。
 * 必须在 main 的初始化阶段、进入 while(1) 之前调用一次。 */
void modbus_init(void);

/* PC 侧（USART1）消费者：主循环轮询调用，取帧 → 校验 → 路由 → 应答 */
void modbus_poll_pc_frames(void);

/* RS485 侧（USART2）消费者：取帧后原样回显给 PC
 * （帧来自总线上的外部变送器，本机不改内容、不做校验） */
void modbus_poll_rs485_frames(void);

#endif /* __MODBUS_RTU_H__ */
