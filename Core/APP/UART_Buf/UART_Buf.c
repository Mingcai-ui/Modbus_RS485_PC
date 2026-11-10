#include "uart_buf.h"
#include "main.h"
#include "usart.h"
#include "stdio.h"

uint8_t uart1_RxData;           // 串口1接收缓冲区
uint8_t uart2_RxData;           // 串口2接收缓冲区

char PC_RX[BUFFERSIZE];         /* UART1中断写入的原始接收区 */
char ModbusRx[BUFFERSIZE];      /* UART2中断写入的原始接收区 */

volatile uint8_t DataCounter_PC;       /* 接收数据计数器 */
volatile uint8_t DataCounter_Modbus;       /* 接收数据计数器 */
volatile uint8_t frameOverflow_PC;     /* USART1 本帧已超出缓冲区，尾部字节被丢弃 */

/*  串口接收中断回调 (每收到 1 个字节触发一次) */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if(DataCounter_PC < BUFFERSIZE)
        {
            PC_RX[DataCounter_PC++] = uart1_RxData;
        }
        else
        {
            /* 缓冲区写满，本帧后续字节无处可放。
             * 这个事实只有中断知道：主循环只能看到 len = 255，
             * 与一条合法的 255 字节长帧完全无法区分，所以必须在这里置标志。 */
            frameOverflow_PC = SET;
        }

        RxInterruptFlag_PC = SET;   /* 标记总线处于接收中 */
        uartTimeCounter_PC = 0;     /* 每收到一个字节就重置帧间隔计时 */
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart1_RxData, 1);
    }
    if (huart->Instance == USART2)
    {
        /* 超长帧不再覆盖旧数据，冻结计数等超时判帧后被丢弃，
        * 避免 DataCounter 回绕导致帧长错乱 */
        if (DataCounter_Modbus < BUFFERSIZE)
        {
            ModbusRx[DataCounter_Modbus++] = uart2_RxData;
        }

        RxInterruptFlag = SET;   /* 标记总线处于接收中 */
        uartTimeCounter = 0;     /* 每收到一个字节就重置帧间隔计时 */
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart2_RxData, 1);
    }
}

/* 串口错误回调：不清错误标志并重启接收，一次噪声就会永久收不到数据 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        
        DataCounter_PC    = 0;
        RxInterruptFlag_PC = RESET;
        uartTimeCounter_PC = 0;
        /* 本帧已被放弃，超长证据一并作废：否则 RxInterruptFlag_PC=RESET 会让
         * 定时器不再判帧，handler 永远不跑，这个 SET 就残留到下一帧，
         * 把一条完全合法的帧误判成 FRAME_Too_LONG。 */
        frameOverflow_PC   = RESET;

        HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart1_RxData, 1);
    }
    if (huart->Instance == USART2)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        
        DataCounter_Modbus    = 0;
        RxInterruptFlag = RESET;
        uartTimeCounter = 0;

        HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart2_RxData, 1);
    }

}

/* printf 重定向到 USART1（单字符，阻塞发送） */
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 0xffff);
  return len;
}