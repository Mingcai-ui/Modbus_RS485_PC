/*
 * critical.c —— 临界区封装（裸机实现）
 *
 * 【当前状态：骨架已就位，尚未被任何模块调用】
 *   现有代码的共享区（frame 层的帧槽）靠"中断只写、主循环只取"的单向
 *   数据流已经够安全，不需要关中断。真正必须用临界区保护的是 M5 的
 *   请求-应答状态机：主循环要主动发帧并等应答，收发双方都会改同一份
 *   状态字段，中间一旦被 SysTick 或 UART 中断插入就会读到半更新状态。
 *   所以先把工具做好，等 M5 再接入。
 */
#include "critical.h"
#include "stm32f1xx_hal.h"   /* 提供 CMSIS 的 __disable_irq/__enable_irq */

/* 【为什么要 depth 计数，而不是直接 __disable_irq/__enable_irq】
 *   如果函数 A 进了临界区，内部又调用了同样进临界区的函数 B，
 *   B 退出时若直接开中断，就会把 A 还没做完的保护提前解除。
 *   用计数器记住嵌套层数，只有最外层退出时才真正开中断。
 *   这与 FreeRTOS 的 taskENTER_CRITICAL 嵌套语义一致，
 *   将来换成 RTOS 版本时调用方代码不用改。 */
static volatile uint32_t critical_depth = 0;

/* 进入临界区：关中断 + 层数加一。
 * 关中断会同时屏蔽 SysTick，所以临界区内不能有耗时操作，
 * 更不能调用 HAL_Delay()（它靠 SysTick 计时，会死等）。 */
void critical_enter(void)
{
    __disable_irq();    //屏蔽中断
    critical_depth++;
}

/* 退出临界区：层数减一，减到 0 才真正开中断。
 * 加 depth>0 的判断是为了防止误用（exit 次数多于 enter）时下溢成
 * 0xFFFFFFFF，那样就永远开不了中断了。 */
void critical_exit(void)
{
    if (critical_depth > 0)
    {
        critical_depth--;
        if (critical_depth == 0)
        {
            __enable_irq();
        }
    }
}
