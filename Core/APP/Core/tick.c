/*
 * tick.c —— 1ms 节拍分发器
 *
 * 【为什么需要这一层】
 *   重构前：SysTick_Handler 里直接写 uartTimer(); uartTimer_PC();
 *   → 中断文件必须 #include "modbus_rtu.h"，必须知道"Modbus"这个具体模块存在。
 *     这是典型的"底层反向依赖上层"：换协议、删协议都得去改中断文件。
 *
 *   重构后：SysTick_Handler 只调 tick_handle()，谁需要 1ms 谁自己来登记。
 *   → 中断文件只认识 tick，不认识任何业务模块；业务模块也只认识 tick。
 *     依赖关系从 "A 认识 B" 变成 "A 和 B 都只认识 tick"，从而互相解耦。
 *
 * 【为什么在中断里遍历函数指针是安全的】
 *   本工程的 1ms 回调（目前只有 frame_tick）都是"遍历几个字段、做几次
 *   比较赋值"级别的极短操作，耗时在微秒量级，远小于 1ms 节拍，不会拖垮
 *   实时性。★ 若将来有耗时回调（如软件延时、阻塞发送），必须自己减频
 *   （每 N 次 tick 执行一次），不要直接塞进这里。
 *
 * 【迁 RTOS 时怎么改】
 *   裸机：SysTick_Handler → tick_handle()
 *   RTOS：每个回调换成一个 osTimer / 任务通知，本文件可以整体删掉，
 *         而注册方（frame.c、modbus_rtu.c）完全不用改 —— 这就是留这层的目的。
 */
#include "tick.h"

/* 回调表容量。当前只用了 2 个（frame_tick 1 个），
 * 留到 8 是为传感器采样、超时计时等后续模块预留；满了会静默丢弃，
 * 调试时如果发现某个回调不执行，先怀疑是不是超了这里。 */
#define TICK_CB_MAX 8

static tick_cb_t tick_cbs[TICK_CB_MAX];
static uint8_t tick_cb_count = 0;

/* 注册一个 1ms 回调。
 * 只需在初始化阶段（main 的 while(1) 之前）调用一次，运行中增删不安全。
 * 传 NULL 会被挡掉，避免运行时判空遗漏导致 HardFault。 */
void tick_register(tick_cb_t cb)
{
    if (cb != NULL && tick_cb_count < TICK_CB_MAX)
    {
        tick_cbs[tick_cb_count++] = cb;
    }
}

/* 由 SysTick_Handler 每 1ms 调用一次，按注册顺序依次执行。 */
void tick_handle(void)
{
    uint8_t i;
    for (i = 0; i < tick_cb_count; i++)
    {
        if (tick_cbs[i] != NULL)
        {
            tick_cbs[i]();  //执行函数指针指向的函数    未加括号数组存函数地址,加括号后
        }
    }
}
