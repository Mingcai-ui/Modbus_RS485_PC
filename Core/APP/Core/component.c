/*
 * component.c —— 主循环组件注册表
 *
 * 【当前状态：骨架已就位，尚未被 main 调用】
 *   M1 只把接口立起来，等 M3/M4 把"采样"、"显示"、"协议轮询"拆成独立
 *   组件后再逐个接入，那时 main.c 的 while(1) 会变成这样：
 *
 *       while (1) { component_loop_all(); }
 *
 *   好处：新增一个业务模块（比如加个继电器控制）只需实现 component_t
 *   并 component_register() 一次，main.c 一行都不用改 —— 这是"方便
 *   后续加模块"的直接落点。
 *
 * 【为什么现在不接入】
 *   接入需要改动 main.c 的 while(1) 结构，属于行为调整，
 *   与本里程碑"行为等价"的约束冲突，故延后。
 */
#include "component.h"

/* 组件表容量。16 对当前规模足够；满了会静默丢弃，
 * 新增组件后若发现不执行，先查这里。 */
#define COMPONENT_MAX 16

static const component_t *comp_list[COMPONENT_MAX];
static uint8_t comp_count = 0;

/* 注册一个组件。只需在初始化阶段调用，运行中增删不安全。
 * 存的是指针，所以 component_t 本身必须是 static/全局生命周期，
 * 不能是栈上的局部变量。 */
void component_register(const component_t *c)
{
    if (c != NULL && comp_count < COMPONENT_MAX)
    {
        comp_list[comp_count++] = c;
    }
}

/* 按注册顺序执行所有组件的 init()。
 * 组件允许只实现 init 不实现 loop（显示类）或反之（轮询类），
 * 所以这里必须逐个判空，不能默认两者都非空。 */
void component_init_all(void)
{
    uint8_t i;
    for (i = 0; i < comp_count; i++)
    {
        if (comp_list[i]->init != NULL)
        {
            comp_list[i]->init();
        }
    }
}

/* 按注册顺序执行所有组件的 loop()。
 * ★ 顺序即优先级：耗时的组件（如 OLED 刷屏）应注册在后面，
 *   否则会拖慢注册在它之后的协议处理。 */
void component_loop_all(void)
{
    uint8_t i;
    for (i = 0; i < comp_count; i++)
    {
        if (comp_list[i]->loop != NULL)
        {
            comp_list[i]->loop();
        }
    }
}
