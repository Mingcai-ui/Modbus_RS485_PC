---
类型: 代码
状态: 归档
标签: [stm32, oled, i2c, 显示, 软件i2c, 可移植]
模块: OLED
硬件平台: STM32F103C8T6
版本: v1.0
创建: 2026-09-17
---

# 代码介绍：OLED 显示模块（简化版）

> **模块类型**：OLED（0.96" 128x64，SSD1306）显示驱动，软件 I2C
> **硬件平台**：STM32F103C8T6（HAL 库）
> **源码位置**：`08_归档/可移植模块代码/OLED/`（`OLED.c` / `OLED.h` / `OLED_Data.c` / `OLED_Data.h`）
> **版权**：江协科技（jiangxiekeji.com），开源库原样保留，不补注释。

## 1. 模块概述
**一句话功能**：软件 I2C 驱动 SSD1306，带显存缓冲 + 全字库（ASCII/中文）+ 绘图函数。
**设计模式**：软件模拟 I2C（GPIO 位翻转）+ 显存（GRAM）缓冲 `OLED_Update` 整屏刷新。
**复杂度**：★★★☆☆（功能多，但接口直观）

## 2. 硬件信息
| 设备 | 端口 | 引脚 | 模式 | 备注 |
|---|---|---|---|---|
| OLED SCL | — | `OLED_SCL_Pin` | 开漏输出 | 无内部上拉 |
| OLED SDA | — | `OLED_SDA_Pin` | 开漏输出 | 同左 |

- CubeMX 配置：GPIO Output **Open Drain**，Pull-up 选 **No pull-up/pull-down**，引脚命名 `OLED_SCL` / `OLED_SDA`。
- 需外接 4.7k~10k 上拉（模块板一般自带）。

## 3. 文件清单
| 文件 | 路径 | 备注 |
|---|---|---|
| 头文件 | [OLED.h](OLED.h) | 引脚宏 + 函数声明 + `OLED_8X16`/`OLED_6X8` 字号宏 |
| 源文件 | [OLED.c](OLED.c) | I2C 时序 + 显存 + 全部显示/绘图函数 |
| 字模数据 | [OLED_Data.c](OLED_Data.c) / [OLED_Data.h](OLED_Data.h) | ASCII/汉字字模 + 图像数据声明 |
| GPIO 配置 | 不入库 | CubeMX 生成 |

## 4. 接口函数（常用摘录）
| 函数 | 频率 | 作用 |
|---|---|---|
| `OLED_Init()` | 初始化一次 | 复位 + 初始化 SSD1306 |
| `OLED_Clear()` | 显示前 | 清显存（需配合 Update 生效） |
| `OLED_ShowString(x,y,str,font)` | 任意 | 显示字符串（font: `OLED_8X16`/`OLED_6X8`） |
| `OLED_ShowNum(x,y,num,len,font)` / `ShowFloatNum` / `ShowSignedNum` / `ShowHexNum` / `ShowBinNum` | 任意 | 各种数值显示 |
| `OLED_Update()` | **显示后必调** | 把显存推到屏幕 |
| `OLED_UpdateArea(x,y,w,h)` | 局部刷新 | 仅刷新区域，加快速度 |
| `OLED_ShowImage(x,y,w,h,img)` | 任意 | 显示图片数据 |
| `OLED_DrawLine/Circle/...` | 任意 | 绘图函数族 |
| `OLED_Printf(x,y,font,fmt,...)` | 任意 | 格式化显示 |

> **关键**：所有 `Show*`/`Draw*` 只写**显存**，必须再调 `OLED_Update()`（或 `UpdateArea`）才上屏。

## 5. 调用流程
```c
// ① main 开头
OLED_Init();
OLED_Clear();

// ② while(1) 刷新显示
while (1) {
    OLED_ShowString(0, 0, "Hello", OLED_8X16);
    OLED_ShowNum(0, 2, 123, 3, OLED_8X16);
    OLED_ShowFloatNum(0, 4, 3.14, 2, 2, OLED_8X16);
    OLED_Update();          // 整屏上屏
}
```
- 局部刷新提速：`OLED_ClearArea(x,y,w,h); OLED_ShowString(...); OLED_UpdateArea(x,y,w,h);`

## 6. 关键宏
| 宏 | 值 | 含义 |
|---|---|---|
| `OLED_8X16` / `OLED_6X8` | 8 / 6 | 字号，兼作横向字符偏移 |
| `OLED_UNFILLED` / `OLED_FILLED` | 0 / 1 | 图形是否填充 |
| `OLED_CHARSET_GB2312`（OLED_Data.h） | — | 汉字字模字符集（UTF8/GB2312 二选一） |
| `SCL_PIN/SDA_PIN/SCL_GPIOX/SDA_GPIOX` | — | 复用 `OLED_SCL_Pin` 等，换引脚改 CubeMX 命名即可 |

## 7. 设计要点
- **显存缓冲**：避免逐点刷屏闪烁，一整帧一次更新。
- **软件 I2C**：任意两个 GPIO 即可驱动，无需硬件 I2C 外设，可移植性强。
- **I2C 底层函数公开**：`OLED_I2C_Start/Stop/SendByte/ReadByte` 可复用于 MPU6050 等外部 I2C 器件。
- ⚠️ 引脚命名固定 `OLED_SCL`/`OLED_SDA`，CubeMX 里按此命名即自动适配，不要改宏。

## 8. 排坑
| 现象 | 原因 | 解决 |
|---|---|---|
| 白屏/花屏 | I2C 上拉不足或引脚未配开漏 | 核对开漏输出 + 外接上拉 |
| 改了不显示 | 忘了 `OLED_Update` | 写显存后必调 Update |
| 汉字乱码 | 字符集宏选错 | `OLED_Data.h` 里 `GB2312/UTF8` 二选一 |
| 刷新太慢 | 整屏 Update 频繁 | 用 `UpdateArea` 局部刷新 |

## 相关
- 详细版见 [[OLED显示_软件I2C驱动]]
- 配合：[[DHT11温湿度_单总线非阻塞驱动]]（显示温湿度）
- 原始代码：`08_归档/可移植模块代码/OLED/`