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
#include "frame.h"
#include "console.h"
#include "dht11.h"
#include "stdio.h"
#include "OLED.h"
#include "dht11.h"
#include "Mb_port.h"
#include "mpu6050.h"
#include "tick.h"
#include "register_map.h"
#include "diag.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
DHT11_Handle_t dht;
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
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  mb_port_init();
  modbus_init();      /* 寄存器表 / 诊断计数清零（原 ModbusRegister 由本层接管） */

  /* Modbus 判帧定时：注册为 1ms tick 回调 */
  tick_register(frame_tick);

  /*  启动串口中断接收  */
  frame_init();
  console_init(); //重定向printf内容，暂未写

  OLED_Init();
  OLED_Clear();
  DHT11_Init(&dht);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  OLED_Clear();
  OLED_ShowString(0, 0, "bad_SHORT:",OLED_6X8);
  OLED_ShowString(0, 8, "bad_LONG:",OLED_6X8);
  OLED_ShowString(0, 16, "bad_CRC:",OLED_6X8);
  OLED_ShowString(0, 24, "bad_Addrs:",OLED_6X8);
  OLED_Update();
  while (1)
  {
    DHT11_Status st = DHT11_Handler(&dht);

    modbus_poll_pc_frames();       // PC 侧数据处理
    modbus_poll_rs485_frames();    // Modbus 侧数据处理

    if (st == DHT11_OK)
    {
      /* B1 修复：原写法 (uint16_t)x * 10.0f 是"先截断再乘"——
       * 25.6 先变 25，再乘 10 得 250（正确值应为 256）。
       * 必须先乘、加 0.5 四舍五入、最后取整。 */
      (void)reg_write(0, (uint16_t)(DHT11_GetTemp(&dht) * 10.0f + 0.5f));
      (void)reg_write(1, (uint16_t)(DHT11_GetHumi(&dht) * 10.0f + 0.5f));
    }
    /*OLED显示会影响串口的接发送*/
    // OLED_ShowNum(60, 0, diag_get(DIAG_FRAME_SHORT), 2, OLED_6X8);   //显示短错误次数
    // OLED_ShowNum(60, 8, diag_get(DIAG_FRAME_LONG), 2, OLED_6X8);    //显示长错误次数
    // OLED_ShowNum(60, 16, diag_get(DIAG_CRC_ERR), 2, OLED_6X8);      //显示CRC错误次数
    // OLED_ShowNum(60, 24, diag_get(DIAG_BAD_ADDR), 2, OLED_6X8);     //显示地址错误次数
    // OLED_Update();
  }

 /* DHT11_LOG_ENABLE */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
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
