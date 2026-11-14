#ifndef __TICK_H__
#define __TICK_H__

#include <stdint.h>
#include <stddef.h>   /* NULL */

/*
 * tick.h —— 1ms 节拍分发器（预留 RTOS 接口）
 *
 * 【解决的依赖问题】
 *   重构前：stm32f1xx_it.c 的 SysTick_Handler 直接调 uartTimer()、uartTimer_PC()，
 *           中断文件必须 include modbus_rtu.h —— 底层反向依赖上层。
 *   重构后：SysTick_Handler 只调 tick_handle()，需要节拍的模块自己来登记。
 *           中断文件与业务模块互不认识，都只认识 tick。
 *
 * 【用法】
 *   1) 模块自己实现一个 void xxx_tick(void) 回调；
 *   2) 在 main 初始化阶段（while(1) 之前）调用 tick_register(xxx_tick)；
 *   3) 回调里做极短的事，耗时操作必须自己降频（每 N 次执行一次）。
 *
 *   例：tick_register(frame_tick);   // 让帧层每 1ms 判一次静默
 *
 * 【约束】
 *   - 回调运行在 SysTick 中断上下文：不能阻塞、不能调用 HAL_Delay()、
 *     不能做长耗时运算，否则直接拖慢整个系统节拍；
 *   - 表容量为 TICK_CB_MAX（见 tick.c），满了静默丢弃；
 *   - 只在初始化阶段注册，运行中增删不安全。
 *
 * 【迁 RTOS】
 *   SysTick 那块换成 osTimer / 任务通知即可，注册方代码不用改。
 */
typedef void (*tick_cb_t)(void);    //定义一个无返回值的无传参的函数指针

void tick_register(tick_cb_t cb);
void tick_handle(void);   /* 由 SysTick_Handler 调用 */

#endif /* __TICK_H__ */
