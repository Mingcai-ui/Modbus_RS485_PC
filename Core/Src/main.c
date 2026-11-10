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
#include "MPU6050.h"
#include "UART_Buf.h"
#include "OLED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---------------- MPU6050 显示节奏 ----------------
 * 采样和刷屏分开定周期：读 14 字节很快（≈1.3ms），刷一屏很慢（≈92ms）。
 * 采样快、刷屏慢，屏上看到的才是「最近的数」，而不是半秒前的旧值。 */
#define MPU_SAMPLE_MS     100U   /* 采样周期 100ms，内部数据 10Hz 刷新 */
#define MPU_REFRESH_MS    500U   /* 刷屏周期 500ms，对应约 92ms/次的 I2C 阻塞 */
#define MPU_INIT_RETRY    3U     /* 上电初始化重试次数 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_RawData_t mpu6050;           /* 六轴传感器（I2C1/PB8-PB9，与 OLED 同总线），原始数据 */
static uint32_t   last_sample_tick;  /* 上次采样时刻 */
static uint32_t   last_refresh_tick; /* 上次刷屏时刻 */
static uint8_t    mpu_ok;            /* 1 = MPU6050 初始化成功 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void MPU6050_ShowRaw(void);
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

  /* 先初始化屏、再初始化传感器：传感器出问题时，报错信息才有地方显示。
   * 两颗器件在同一条 I2C1 上：OLED 0x3C、MPU6050 0x68，地址不冲突。 */
  OLED_Init();
  OLED_Clear();
  OLED_ShowString(0, 24, "MPU6050...", OLED_8X16);
  OLED_Update();

  /* MPU6050 初始化：重试 3 次，失败也继续往下跑。
   *
   * 这里没有沿用 while(1) 死等——死等的后果是整机卡住：屏不刷新、Modbus 不应答，
   * 上板时你分不清是屏坏了、线松了、还是传感器本身坏了。失败就降级成「屏上写一行
   * 错误 + 主循环照常跑」，让屏自己把现场情况说出来。 */
  for (uint8_t retry = 0; retry < MPU_INIT_RETRY; retry++)
  {
    if (MPU6050_Init(&hi2c1) == 0)
    {
      mpu_ok = 1;
      break;
    }
    HAL_Delay(200);
  }

  OLED_Clear();
  if (mpu_ok)
  {
    OLED_ShowString(0, 24, "MPU6050 OK", OLED_8X16);
  }
  else
  {
    /* 把 WHO_AM_I 一起读出来显示，能直接区分两种故障：
     * 0x68 = 芯片在、只是配置没成功（多半 I2C 时序/上拉问题）
     * 0x00 / 0xFF = 总线根本没通（查 SDA/SCL 接线、查 4.7k 上拉） */
    OLED_ShowString(0,  8, "MPU6050 FAIL", OLED_8X16);
    OLED_ShowString(0, 32, "ID:", OLED_8X16);
    OLED_ShowHexNum(24, 32, MPU6050_GetID(&hi2c1), 2, OLED_8X16);
  }
  OLED_Update();

  last_sample_tick  = HAL_GetTick();
  last_refresh_tick = HAL_GetTick();
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uartDataHandler_PC();
    uartDataHandler_Modbus();

    /* ---------------- 1) 采样 ----------------
     * 100ms 一次。硬件 I2C 读 14 字节约 1.3ms，全程不关中断——这一点和 DHT11
     * 完全不同：DHT11 是单总线、靠关中断凑微秒级时序，必须等总线空闲才敢碰；
     * I2C 是外设自己在跑，随时可以发起，不用让路。 */
    if (HAL_GetTick() - last_sample_tick >= MPU_SAMPLE_MS)
    {
      last_sample_tick = HAL_GetTick();
      if (mpu_ok)
      {
        MPU6050_ReadRaw(&hi2c1, &mpu6050);
      }
    }

    /* ---------------- 2) 刷屏 ----------------
     * 刷一屏要写 8 页 × 128 字节，100kHz 下约 92ms，这段时间是阻塞的。
     * 两条约束：
     *   1. 周期不能太小，否则主循环一大半时间卡在 I2C 里；
     *   2. 两路串口任一路正在收帧就先不刷，把这 92ms 让给 Modbus
     *      （沿用 DHT11 那版的取舍：宁可掉屏刷新率，不可顶掉 PC 的帧）。
     * PC 若一直高频轮询，屏会一直不刷新，属预期行为。 */
    if (mpu_ok
        && RxInterruptFlag_PC != SET
        && RxInterruptFlag    != SET
        && (HAL_GetTick() - last_refresh_tick >= MPU_REFRESH_MS))
    {
      last_refresh_tick = HAL_GetTick();
      MPU6050_ShowRaw();
    }
  }
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
/**
  * @brief  把一帧 MPU6050 原始数据刷到 OLED
  * @note   8X16 字体：每字符 8px 宽、16px 高，128px 宽 = 16 字符/行，64px 高 = 4 行
  *
  *             X=0        X=64
  *           ┌──────────┬──────────┐
  *    Y=0    │AX+01234  │AY-00012  │
  *    Y=16   │AZ+16384  │GX+00045  │
  *    Y=32   │GY-00008  │GZ+00001  │
  *    Y=48   │T+27.53C  │          │
  *           └──────────┴──────────┘
  *
  *   每个字段 = 标签 2 字符(16px) + 符号 1 位 + 5 位数字(48px) = 64px，
  *   两个字段正好铺满 128px。位数固定 ⇒ 刷新时不用先清屏，直接覆盖不会有残影。
  */
static void MPU6050_ShowRaw(void)
{
  int32_t t_x100;   /* 温度 ×100，例：27.53℃ → 2753 */
  int32_t t_frac;   /* 小数部分（取绝对值） */

  /* 温度换算 T = Temp_RAW / 340 + 36.53。整个式子放大 100 倍走整数运算，
   * 既避开 %f，也避开软浮点库：3653 就是 36.53 × 100。 */
  t_x100 = (int32_t)mpu6050.Temp_RAW * 100 / 340 + 3653;

  /* C 的整数除法向零截断，取余会带符号：-1230 / 100 = -12，%100 = -30，
   * 所以小数部分必须取绝对值，否则 -30 强转 uint32_t 会变成 4294967266。 */
  t_frac = t_x100 % 100;
  if (t_frac < 0)
  {
    t_frac = -t_frac;
  }

  /* 第一行：AX / AY */
  OLED_ShowString    (  0,  0, "AX", OLED_8X16);
  OLED_ShowSignedNum ( 16,  0, mpu6050.Accel_X, 5, OLED_8X16);
  OLED_ShowString    ( 64,  0, "AY", OLED_8X16);
  OLED_ShowSignedNum ( 80,  0, mpu6050.Accel_Y, 5, OLED_8X16);

  /* 第二行：AZ / GX。静止平放时 AZ 应约 +16384（±2g 量程下 1g = 16384） */
  OLED_ShowString    (  0, 16, "AZ", OLED_8X16);
  OLED_ShowSignedNum ( 16, 16, mpu6050.Accel_Z, 5, OLED_8X16);
  OLED_ShowString    ( 64, 16, "GX", OLED_8X16);
  OLED_ShowSignedNum ( 80, 16, mpu6050.Gyro_X, 5, OLED_8X16);

  /* 第三行：GY / GZ。静止不动时三个角速度都应在 0 附近小幅跳动 */
  OLED_ShowString    (  0, 32, "GY", OLED_8X16);
  OLED_ShowSignedNum ( 16, 32, mpu6050.Gyro_Y, 5, OLED_8X16);
  OLED_ShowString    ( 64, 32, "GZ", OLED_8X16);
  OLED_ShowSignedNum ( 80, 32, mpu6050.Gyro_Z, 5, OLED_8X16);

  /* 第四行：温度。手拼整数/小数点/小数，不用 OLED_ShowFloatNum
   * （那个形参是 double，会牵进软浮点运算） */
  OLED_ShowString    (  0, 48, "T", OLED_8X16);
  OLED_ShowSignedNum (  8, 48, t_x100 / 100, 2, OLED_8X16);
  OLED_ShowChar      ( 32, 48, '.', OLED_8X16);
  OLED_ShowNum       ( 40, 48, (uint32_t)t_frac, 2, OLED_8X16);
  OLED_ShowString    ( 56, 48, "C", OLED_8X16);

  /* 显存改完才真正发送，上面十几个函数都只是改内存数组 */
  OLED_Update();
}
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
