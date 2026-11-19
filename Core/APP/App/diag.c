/*
 * diag.c —— 诊断计数器实现
 */
#include "diag.h"
#include <stddef.h>

/* 计数值。只在主循环上下文访问，故不加 volatile（原因见 diag.h）。 */
static uint32_t diag_cnt[DIAG_COUNT];

// /* 名字表：下标与 diag_id_t 一一对应。
//  * 加新计数器时这里必须同步加一行，否则返回 "?" —— 编译不会报错，
//  * 这是本文件唯一需要人工保证的一致性。 */
// static const char *const diag_names[DIAG_COUNT] =
// {
//     /* DIAG_FRAME_SHORT */ "SHORT",
//     /* DIAG_FRAME_LONG  */ "LONG",
//     /* DIAG_CRC_ERR     */ "CRC",
//     /* DIAG_BAD_ADDR    */ "ADDR",
// };

void diag_inc(diag_id_t id)     //id就是数组空间的下标，用于计数器的自增
{
    if ((unsigned)id < DIAG_COUNT)
    {
        diag_cnt[id]++;
    }
}

uint32_t diag_get(diag_id_t id)     //将指定计数器信息返回
{
    if ((unsigned)id >= DIAG_COUNT)
    {
        return 0;
    }

    return diag_cnt[id];
}

// const char *diag_name(diag_id_t id)
// {
//     if ((unsigned)id >= DIAG_COUNT)
//     {
//         return "?";
//     }

//     return diag_names[id];
// }

void diag_reset(void)   //将所有计算器清零
{
    uint32_t i;

    for (i = 0; i < DIAG_COUNT; i++)
    {
        diag_cnt[i] = 0;
    }
}
