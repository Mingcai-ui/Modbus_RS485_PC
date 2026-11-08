/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbus_rtu.h"
#include "UART_Buf.h"
#include "dht11.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* DHT11 调试输出开关：1 = 每 2s 从 USART1 打印一次读数，0 = 仅采样不打印。
 *
 * 注意：USART1 同时是 PC 侧 Modbus 链路，printf 输出的是 ASCII 文本，
 * 若与 Modbus 二进制帧交叠，PC 端会看到乱码、CRC 校验必错。
 * 因此打印前判断 RxInterruptFlag_PC == RESET（PC 侧总线静默）才输出。
 *
 * 2026-11-08：单总线驱动已上板验证通过（raw=3B 00 1B 05 5B，H=59.0% T=27.5℃，
 * err 恒 0），故置 0。需要再观测时改回 1，并把 dht11.h 的 DHT11_DEBUG_TIMES
 * 一并改回 1，才能打印原始字节与位宽。 */
#define DHT11_LOG_ENABLE   0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
DHT11_Handle_t dht11;      /* 本地温湿度传感器（PA11），后续作为 Modbus 从机地址 02 的数据源 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(RS485_GPIO_Port, RS485_Pin, GPIO_PIN_RESET);

  /*  启动串口中断接收  */
  HAL_UART_Receive_IT(&huart1, &uart1_RxData, 1);
  HAL_UART_Receive_IT(&huart2, &uart2_RxData, 1);

  /*  DHT11 初始化（必须在 SystemClock_Config 之后，DWT 换算依赖 SystemCoreClock） */
  DHT11_Init(&dht11);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uartDataHandler_PC();
    uartDataHandler_Modbus();

#if DHT11_LOG_ENABLE
    /* DHT11 观测：每 2s 读一次，结果经 USART1 打印给 PC
     *
     * 打印时机：只在 PC 侧总线静默时输出，避免 ASCII 混进 Modbus 二进制帧。
     * 但 printf 是阻塞发送，若此刻 PC 正好开始发帧仍可能被打乱，
     * 所以观测时把串口助手设为「只看不发」，不要同时用 Modbus Poll 轮询。 */
    {
        DHT11_Status dht_st = DHT11_Handler(&dht11);

        if (dht_st == DHT11_OK)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);   /* LED 同步翻转，双重指示 */

            if (RxInterruptFlag_PC != SET)
            {
                /* 用整数部分拼接，不用 %f：链接默认可能未开 -u _printf_float，
                 * 那样 %f 会静默输出空白，比报错更难查 */
                printf("DHT11 T=%d.%dC  H=%d.%d%%RH\r\n",
                       dht11.temp_int, dht11.temp_dec,
                       dht11.humidity_int, dht11.humidity_dec);
#if DHT11_DEBUG_TIMES
                /* 排障附加信息：需同时把 dht11.h 的 DHT11_DEBUG_TIMES 置 1 */
                printf("  raw: %02X %02X %02X %02X %02X\r\n",
                       DHT11_DbgRaw[0], DHT11_DbgRaw[1], DHT11_DbgRaw[2],
                       DHT11_DbgRaw[3], DHT11_DbgRaw[4]);
                printf("  bit0-11 us: %u %u %u %u %u %u %u %u %u %u %u %u\r\n",
                       DHT11_DbgBitUs[0],  DHT11_DbgBitUs[1],  DHT11_DbgBitUs[2],
                       DHT11_DbgBitUs[3],  DHT11_DbgBitUs[4],  DHT11_DbgBitUs[5],
                       DHT11_DbgBitUs[6],  DHT11_DbgBitUs[7],  DHT11_DbgBitUs[8],
                       DHT11_DbgBitUs[9],  DHT11_DbgBitUs[10], DHT11_DbgBitUs[11]);
#endif
            }
        }
        else if (dht_st == DHT11_ERROR && RxInterruptFlag_PC != SET)
        {
            printf("DHT11 FAIL\r\n");
        }
    }
#else
    /* 正常运行时只采样、不打印：每 2s 一次，成功读到就翻转 PC13 LED 作心跳。
     * 下一步接入 Modbus 地址 02 时，把 dht11 的四个字段写入 localReg[]。 */
    if (DHT11_Handler(&dht11) == DHT11_OK)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
#endif /* DHT11_LOG_ENABLE */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
