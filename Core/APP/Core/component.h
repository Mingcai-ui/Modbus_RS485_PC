#ifndef __COMPONENT_H__
#define __COMPONENT_H__

#include <stdint.h>
#include <stddef.h>   /* NULL */

/*
 * component.h —— 主循环组件注册表（预留 RTOS 接口）
 *
 * 【解决什么问题】
 *   原来所有业务都堆在 main.c 的 while(1) 里（采样、协议轮询、OLED 刷新），
 *   加一个功能就要改 main.c，越改越长。改成"组件制"后，
 *   每个业务模块自己声明一个 component_t 并注册，main.c 只负责遍历。
 *
 * 【用法】三步：
 *   1) 在模块 .c 里实现 init/loop（不需要的那个填 NULL）；
 *   2) 定义 static const component_t xxx_comp = {
 *          .name = "dht11", .init = dht_init, .loop = dht_loop };
 *   3) 初始化阶段调用 component_register(&xxx_comp);
 *
 *   之后 main 的循环体只剩：
 *       component_init_all();      // 初始化阶段调一次
 *       while (1) { component_loop_all(); }
 *
 * 【约束】
 *   - 注册的是指针，component_t 必须是 static/全局生命周期，不能是栈变量；
 *   - 注册顺序即执行顺序，耗时组件（OLED 刷屏）应排在协议处理之后；
 *   - init/loop 都可为 NULL，遍历时逐个判空，所以"只做初始化"或
 *     "只做轮询"的模块也能注册。
 *
 * 【当前状态】骨架已就位，尚未被 main 调用（接入会改动 while(1) 结构，
 *   属于行为调整，留到 M3/M4）。
 *
 * 【迁 RTOS】每个组件对应一个任务，loop() 变成任务体。
 */
typedef struct
{
    const char *name;      /* 组件名（调试用，可为 NULL） */
    void (*init)(void);    /* 初始化，可为 NULL */
    void (*loop)(void);    /* 主循环轮询，可为 NULL */
} component_t;

void component_register(const component_t *c);
void component_init_all(void);   /* 按注册顺序执行所有非空 init */
void component_loop_all(void);   /* 按注册顺序执行所有非空 loop */

#endif /* __COMPONENT_H__ */
