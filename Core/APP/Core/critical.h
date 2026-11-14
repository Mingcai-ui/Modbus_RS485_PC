#ifndef __CRITICAL_H__
#define __CRITICAL_H__

/*
 * critical.h —— 临界区封装（预留 RTOS 接口）
 *
 * 【解决什么问题】
 *   主循环和中断会改同一份共享数据时，若主循环"读—算—写"到一半被中断
 *   插入并改写，就会读到半更新的状态，导致判断错乱。临界区把这段操作
 *   做成不可打断的原子块。
 *
 * 【用法】必须成对出现，且不要跨函数长时间持有：
 *
 *       critical_enter();
 *       ... 只放几行读写共享变量的代码 ...
 *       critical_exit();
 *
 * 【约束】
 *   - 支持嵌套：内部调用另一个也用临界区的函数是安全的（靠 depth 计数）；
 *   - 临界区内禁止耗时操作，尤其禁止 HAL_Delay()（关中断后 SysTick 不走，
 *     会死等）；原则是"能少关一微秒就少关一微秒"。
 *
 * 【当前状态】骨架已就位，尚未被调用；M5 请求-应答状态机会用上。
 *
 * 【迁 RTOS】
 *   换成 taskENTER_CRITICAL()/taskEXIT_CRITICAL() 或互斥量，
 *   调用方的 enter/exit 写法不用改。
 */
void critical_enter(void);
void critical_exit(void);

#endif /* __CRITICAL_H__ */
