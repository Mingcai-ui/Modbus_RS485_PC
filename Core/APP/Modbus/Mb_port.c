#include "mb_port.h"
#include "main.h"
#include "usart.h"

/* 发送超时随帧长走：9600 下每字节 1.04ms，留一倍余量 */
#define MB_TX_TIMEOUT(len)   ((uint32_t)(len) * 2u + 10u)

static void rs485_de(uint8_t tx)
{
    HAL_GPIO_WritePin(RS485_GPIO_Port, RS485_Pin,tx ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void mb_port_init(void)
{
    rs485_de(0);                 /* 上电默认收态，避免占住总线 */
}

int mb_port_send(mb_port_t port, const uint8_t *buf, uint8_t len)
{
    if (buf == NULL || len == 0) return -1;

    switch (port)
    {
    case MB_PORT_RS485:
        rs485_de(1);             /* DE 有效：发送态 */
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, MB_TX_TIMEOUT(len));
        rs485_de(0);             /* 返回时 TC 已置位，末字节已移出，可安全切换 */
        return 0;

    case MB_PORT_PC:
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, MB_TX_TIMEOUT(len));
        return 0;

    default:
        return -1;
    }
}
