# Modbus_RS485_PC — PC ↔ RS485 路由器（01 透传 / 02 本机 DHT11 从机）

> 单片机不再是无脑透传桥，而是一个**带身份判断的路由器**：收到 PC 的 Modbus 请求帧后，
> 先验帧（长度 + CRC）、再看地址、最后决定这帧归谁处理。
>
> - 地址 `0x01` → 原样透传给 RS485 总线上的外部温湿度变送器，由它自己应答
> - 地址 `0x02` → 本机 DHT11 虚拟从机，读缓存组帧后从 USART1 直接回 PC
> - 其他地址 → 静默丢弃并计数
>
> STM32F103C8T6 + HAL，裸机 `while(1)`，无 RTOS。**当前版本 v1.0.0（main，纯透传桥）；
> `feat/modbus-router` 为 v2.0.0 候选**（新增路由与 02 从机）；
> 本分支 `MPU6050_makePacket` 为 v2.1.0 候选，在 v2.0.0 基础上加 MPU6050 采样与 OLED 显示。
> 三者均待上板实测后合并 main 并打 tag。

---

## 1. 这个版本做什么

上一版（v1.0.0）是裸透传桥：**不校验、不解析**，地址错 / CRC 错 / 长度不完整的帧一律照搬。
这一版把「先验帧、再路由」前移到了转发之前：

```
PC 主站 (Modbus Poll / 串口助手)
        │  USART1  115200 8N1   PA9(TX) / PA10(RX)
        ▼
┌───────────────────────────────────────────────────────────┐
│  STM32F103C8T6  路由器                                    │
│                                                           │
│  PC_RX[] ──> uartDataHandler_PC()                        │
│     ├─ checkFrame()：长度 + CRC 判决                       │
│     └─ switch(地址)                                       │
│          ├─ 0x01 → sendMessage() → USART2 → RS485 变送器  │
│          ├─ 0x02 → transmitDataMake() → makePacket_03()   │
│          │         读 ModbusRegister[0..1] → USART1 回 PC │
│          └─ 其他 → bad_Addrs++（丢弃）                     │
│                                                           │
│  MPU6050 ── 主循环 100ms 采样 ──> OLED(0x3C) 显示         │
└───────────────────────────────────────────────────────────┘
        │  USART2  9600 8N1  PA2/PA3 + PA1(DE/RE)
        ▼
RS485 温湿度变送器（从机地址 0x01）
```

> 注：上图里的本机从机数据源已变。`main.c` 现在只做 MPU6050 采样与 OLED 显示，
> DHT11 采样已从主循环移除，`ModbusRegister[0..1]` 暂无人写入（详见更新日志 v2.1.0）。

| | v1.0.0（main） | 本分支（v2.0.0 候选） |
|---|---|---|
| 判帧 | ✅ 5ms 静默 | ✅ 同左 |
| CRC / 长度校验 | ❌ 不做 | ✅ `checkFrame()` 前置 |
| 地址路由 | ❌ 无 | ✅ 01 透传 / 02 本地 / 其他丢弃 |
| 本机从机（DHT11） | ❌ 无 | ✅ 02 只读温湿度 |

## 2. 硬件与参数

| 项 | 配置 |
|---|---|
| MCU | STM32F103C8T6，HSE 8MHz → PLL×9 = 72MHz |
| USART1（PC 侧） | 115200 / 8N1，PA9=TX、PA10=RX，中断收 + 阻塞发 |
| USART2（RS485 侧） | 9600 / 8N1，PA2=TX、PA3=RX，中断收 + 阻塞发 |
| RS485 方向脚 | **PA1**，`SET`=DE 有效(发送)、`RESET`=接收态；`gpio.c` 上电初始为接收态 |
| DHT11 数据脚 | **PA11**，单总线，代码开内部上拉，建议再加 4.7k~10k 外部上拉 |
| 判帧阈值 | `FRAME_GAP_MS = 5`（按 9600 侧 3.5T ≈ 3.65ms 定），SysTick 1ms 滴答 |
| 缓冲区 | `BUFFERSIZE = 255`，两路各自独立 |
| 外部从机 | 温湿度变送器，地址 0x01 |
| 本机从机 | DHT11，地址 0x02，保持寄存器 0/1 |

## 3. 代码结构

| 文件 | 职责 |
|---|---|
| `Core/Src/main.c` | 初始化 + 置 PA1 接收态 + 启动两路 `HAL_UART_Receive_IT` + `OLED_Init` + `MPU6050_Init`（重试 3 次，失败降级为屏上提示）；`while(1)` 轮询两个消费者 + 100ms 采样 MPU6050 + 500ms 刷屏 |
| `Core/APP/UART_Buf/UART_Buf.c/.h` | 两路接收缓冲 `PC_RX[]` / `ModbusRx[]`、字节计数器、`frameOverflow_PC` 溢出标志、`HAL_UART_RxCpltCallback` 分流、`HAL_UART_ErrorCallback` 双路恢复、`_write()` printf 重定向 |
| `Core/APP/Modbus/modbus_rtu.c/.h` | 两路 1ms 判帧 `uartTimer()` / `uartTimer_PC()`、消费者 `uartDataHandler_PC()` / `uartDataHandler_Modbus()`、`checkFrame()` 判决、`sendMessage()`（按地址选出口）、`transmitDataMake()` + `makePacket_*` 功能码分发、`sendException()` 异常响应、四类丢弃计数 |
| `Core/APP/CRC/CRC16.c/.h` | Modbus CRC16 —— `checkFrame()` 与 `makePacket_*` 都调用，多项式 0xA001 反射、初值 0xFFFF |
| `Core/APP/DHT11/dht11.c/.h` | DHT11 单总线驱动（非阻塞调度 + 关中断读取），见 `DHT11/DHT11_介绍.md`。**当前 main 已不再调用**，文件保留待定 |
| `Core/APP/MPU6050/MPU6050.c/.h` | 六轴传感器驱动（I2C1 / PB8-PB9，地址 0x68）：唤醒 + 采样率 + 量程配置，一次 burst 读 14 字节（加速度 6 + 温度 2 + 角速度 6） |
| `Core/APP/OLED/OLED.c/.h` + `OLED_Data.c/.h` | 江协科技 4 针 I2C OLED 驱动（SSD1306，地址 0x3C），移植时把软件 I2C 底层（`OLED_W_SCL/SDA`、`OLED_I2C_*`）整段换成 `HAL_I2C_Mem_Write`，与 MPU6050 共用硬件 I2C1 |
| `Core/Src/stm32f1xx_it.c` | `SysTick_Handler` 挂两个判帧函数；`USART1/2_IRQHandler` → `HAL_UART_IRQHandler` |
| `.eide/eide.yml` / `Modbus_RS485_PC.ioc` | EIDE 编译配置 / CubeMX 引脚与 NVIC 配置 |

## 4. 关键实现点

- **先验帧再路由**：`checkFrame()` 只做「长度 ≥ 4 + CRC」，返回 `FRAME_OK / Too_SHORT / Too_LONG / CRC_ERR`。坏帧在路由前就被拦下，不再占用 RS485 总线和变送器的处理时间。
- **超长与「正好 255 字节合法帧」的区分**：两者 `DataCounter_PC` 都停在 255，主循环看不到区别。由接收中断在缓冲区写满时置 `frameOverflow_PC`，主循环 `uartDataHandler_PC()` 取走并清零，折成 `FRAME_Too_LONG`。`BUFFERSIZE=255` 恰好等于 Modbus 单帧合法上限（0x10 写满 123 寄存器：1+1+2+2+1+246+2=255），**不可升到 256**，否则 `uint8_t len` 回绕成 0。
- **地址决定出口**：`sendMessage()` 看帧头第一个字节——`0x01` 走 USART2 + DE 拉高（RS485），其余地址写 USART1（本机应答回 PC）。因为 Modbus 帧头就是从机地址，这个判断天然成立，`makePacket_*` 和 `sendException` 一行都不用改，全部自动获得按地址选出口的能力。
- **02 只读**：`case 0x02` 只放行 `0x03` 读保持寄存器，其余功能码回异常 `0x01`。因为写进去的值会被下一次 2s 采样直接覆盖，开放写会让 PC 误以为写入成功。
- **采样与打印解耦**：`DHT11_Handler()` 在 `#if DHT11_LOG_ENABLE` 之外无条件跑，打印降级为可选分支。采样成功写 `ModbusRegister[0]=温度×10`、`[1]=湿度×10`（int16 补码 / uint16），冷启动成功前保持 0。
- **采样避开总线忙**：`DHT11_ReadFrame()` 内部关中断约 4ms，主循环只在 `RxInterruptFlag_PC` 与 `RxInterruptFlag` 都静默时才驱动传感器，把这段 4ms 挪到总线空闲缝隙，避免顶掉 PC 帧。
- **两路错误恢复**：`HAL_UART_ErrorCallback()` 对 USART1 / USART2 分别清 ORE、清各自计数器与判帧状态、清 `frameOverflow_PC`、重启接收。USART1 分支补清 `frameOverflow_PC` 是修「溢出标志跨帧残留」的关键。
- **发送超时随帧长动态计算**：`HAL_UART_Transmit()` 超时统一为 `(uint32_t)len * 2 + 10`，9600 下每字节 1.04ms，留一倍余量。
- **两路完全隔离**：各有独立缓冲区、计数器、判帧状态（`*_PC` 后缀）。完成标志由消费者一进来就清，先 `memcpy` 拷出再 `memset` 接收区。

## 5. 实测验证

### 5.1 地址 01 透传（v1.0 已验证）

```
PC  →  01 03 00 01 00 01 D5 CA     请求：读变送器保持寄存器 1 个
PC  ←  01 03 02 01 14 B9 DB       应答：字节数 02 / 数据 0x0114(276)
```

### 5.2 地址 02 本机从机（数据源原为 DHT11，当前已停用）

| 请求帧 | 应答帧 | 含义 |
|---|---|---|
| `02 03 00 00 00 02 C4 38` | `02 03 04 01 13 02 4E B8 5E` | 温度 0x0113=275 → 27.5℃；湿度 0x024E=590 → 59.0%RH |
| `02 03 00 09 00 04 <CRC>` | `02 83 02 30 F1` | 起始 9 + 4 > 10，越界 → 异常 0x02 |
| `02 06 00 00 00 64 <CRC>` | `02 86 01 <CRC>` | 02 只读，写请求 → 异常 0x01 |

> 寄存器布局：`0`=温度×10 的 int16 补码，`1`=湿度×10 的 uint16。温湿度变送器行业通用布局，
> 换 DHT22 / SHT30 也不用改 PC 端。零下温度注意 PC 端要用**有符号**显示，否则 −52 会显示成 65484。
>
> ⚠️ **v2.1.0 起上述数据源已断**：`main.c` 不再驱动 DHT11，`ModbusRegister[0..1]` 无人写入，
> 02 地址仍能正常应答 0x03，但返回的恒为 `00 00`。上表帧样例是 v2.0 采样还在时抓的，留作格式参考。

### 5.3 连续收发速率边界（2026-11-08 实测）

请求固定 `01 03 00 00 00 02 C4 0B`（读 2 寄存器，应答应为 9 字节）：

| PC 发送间隔 | 现象 |
|---|---|
| 16ms（≈62 帧/秒） | 应答被腰斩，只收到前缀 3~5 字节，始终收不全 |
| 100ms | 稳定「发一帧、收一帧」，应答 9 字节完整 |

**成因**：一次事务往返约 62ms（变送器处理 ≈50ms + 9600 下 9 字节应答 9.4ms）。PC 以 16ms 节奏灌入时，
桥收到新帧立即 `rs485Mode(1)` 转发，收发器在发送期间关闭接收，变送器尚未发完的应答字节被物理丢弃。

## 6. 已知限制与待办

| 项 | 说明 |
|---|---|
| **02 端到端实测未做** | 需上板用 Modbus Poll（从站 2 / 功能码 03 / 起始 0 / 数量 2）验证；01 透传因 `sendMessage` 改动需**回归复测** |
| 无请求—应答状态机 | 当前 PC 帧到达即转发，不判断总线是否忙、不排队、不限速。`IDLE → SENDING → WAIT_RESP → IDLE` 状态机是高速请求下不丢字节的唯一正解，本分支不处理 |
| 从机遗留功能码 | `makePacket_01/05/06/15/16` 当前**不调用**（02 只开放 0x03，01 透传不解析），属遗留代码；`transmitDataMake()` + `makePacket_03()` 已被 02 路径使用 |
| 拆包不重组 | Modbus RTU 无长度字段，帧边界靠静默间隔判定。PC 端若把一帧拆成间隔 >5ms 的多段发，会碎成多帧被判 `bad_SHORT`/`bad_CRC`。根治需按功能码「长度字段提前判帧」，本分支不做 |
| 255 字节边界未实测 | 超长帧 → `bad_LONG` 的路径逻辑已通，但恰好 255 字节的合法长帧未上板验证 |
| 注释陈旧 | `modbus_rtu.c` 文件头仍写「PB2 控制方向」，实际为 PA1，待改 |

## 7. 构建

- **EIDE**（本项目实际使用）：打开 `Modbus_RS485_PC.code-workspace`，`.eide/eide.yml` 已配好头文件路径（`Core/APP/Modbus`、`Core/APP/UART_Buf`、`Core/APP/CRC`、`Core/APP/DHT11`），arm-none-eabi-gcc 编译，产物在 `build/`（已忽略）。
- **STM32CubeIDE**：可直接导入 `.project` / `.cproject`。修改引脚后重新生成代码时，注意 `stm32f1xx_it.c` 的 `USER CODE` 区与 NVIC 勾选不要被覆盖。
- 仓库已忽略 `/build`、`/bin`、`/obj`、`/out`、`/.settings` 等产物。

---

## 更新日志

> **规范：每次更新推送，都必须说明三段 —— 改了什么 / 加了什么 / 解决了什么。**
> `git commit` 的 message、推送时的说明、以及此处的每一条记录，都按这个结构写。

### v2.1.0 候选 · 2026-11-10 · `MPU6050_makePacket`（本分支，未合并 main / 未打 tag）

- **改了什么**：`main.c` 的 `USER CODE` 区整体重写——删掉 `dht11.h` / `stdio.h` / `DHT11_LOG_ENABLE` 及其包着的旧串口打印块，主循环改为「100ms 采样 + 500ms 刷屏」两段独立节拍；MPU6050 初始化由死等 `while(MPU6050_Init(...) != 0)` 改为重试 3 次、失败降级为屏上提示；OLED 驱动的软件 I2C 底层（`OLED_W_SCL` / `OLED_W_SDA` / `OLED_GPIO_Init` / `OLED_I2C_Start/Stop/SendByte/ReadByte`）整段删除，`OLED_WriteCommand` / `OLED_WriteData` 改写为 `HAL_I2C_Mem_Write`；`.eide/eide.yml` 增补 `Core/APP/OLED` 头文件路径。
- **加了什么**：`Core/APP/MPU6050/`（驱动：唤醒 + 1kHz 采样率 + ±2g / ±250dps 量程 + 14 字节 burst 读）；`Core/APP/OLED/`（江协科技 4 针 I2C 驱动 4 个文件，SSD1306 @0x3C）；`MPU6050_ShowRaw()` 刷屏函数（8X16 字体铺满 4 行：AX/AY、AZ/GX、GY/GZ、温度；固定位宽所以不用清屏、无残影）；`MPU_SAMPLE_MS` / `MPU_REFRESH_MS` / `MPU_INIT_RETRY` 三个节奏宏；初始化失败时屏上直接显示 `WHO_AM_I` 故障码。
- **解决了什么**：① 三个编译硬伤——`last_mpu_tick` 未声明、`#if DHT11_LOG_ENABLE` 只开一个却配了两个 `#endif`、MPU 打印被 `DHT11_LOG_ENABLE=0` 整段吃掉（"编译过了也一行不打印"）；② 软件 I2C 与硬件 I2C1 抢 PB8/PB9——任何一次 GPIO 输出配置都会把引脚从 AF 模式拽回普通 IO，结果是屏写不进、传感器也读不到，现在统一走硬件 I2C1（屏 0x3C / 传感器 0x68，地址不冲突）；③ 原 OLED 驱动 `OLED_GPIO_Init()` 内部会调用 `MX_GPIO_Init()`，顺带把 RS485 方向脚 / LED / DHT11 引脚重配一遍，已随底层一并删除；④ 传感器未接时整机卡死在初始化、Modbus 不应答且现场无法定位故障；⑤ 刷屏 92ms 阻塞可能顶掉 Modbus 帧——已加「两路串口任一路正在收帧就先不刷」的让路判断（沿用 DHT11 那版的取舍）。

> **验证**：`arm-none-eabi-gcc -Os` 全量编译 + 链接通过，`text 14544 / data 12 / bss 4276`（64KB Flash / 20KB RAM 余量充足），**未上板**。DHT11 驱动文件保留未删，去留待定。

### v2.0.0 候选 · 2026-11-08 · `feat/modbus-router`（本分支，未合并 main / 未打 tag）

- **改了什么**：`sendMessage()` 由「固定走 USART2 + 拉高 DE」改为按帧头地址选出口（0x01 走 RS485，其余写 USART1）；`uartDataHandler_PC()` 由裸转发重构为「溢出检查 → 长度/CRC 判决 → 地址 switch」；`main.c` 把 DHT11 采样抽到 `#if DHT11_LOG_ENABLE` 之外；`MODBUS_CRC16` 形参改 `const char *`；`BUFFERSIZE` 注释订正为「255 = 0x10 写满 123 寄存器的 ADU 上限」。
- **加了什么**：`FrameCheckResult` 枚举与 `checkFrame()`；四类丢弃计数器 `bad_SHORT` / `bad_LONG` / `bad_CRC` / `bad_Addrs`；地址路由 `switch(pcFrame[0])`（01 透传 / 02 只读 0x03 / 其他丢弃）；USART1 接收中断溢出标志 `frameOverflow_PC` 及错误回调补清；`transmitDataMake()` 入口最小帧长校验（<8 字节回异常）；DHT11 单总线驱动（`Core/APP/DHT11/`，PA11，非阻塞调度 + 关中断读取，上板验证 27.5℃/59.0%RH）；采样结果写入 `ModbusRegister[0..1]`。
- **解决了什么**：坏帧（CRC 错 / 帧长 <4 / 地址非法）不再占用 RS485 总线；超长帧与「正好 255 字节合法帧」无法区分的问题（由中断置标志、主循环消费）；`frameOverflow_PC` 跨帧残留导致合法帧被误判 `FRAME_Too_LONG`；02 地址无入口、只能落进 `bad_Addrs`；DHT11 采样与调试打印耦合导致寄存器永远为 0；02 可被 0x06 写入的漏洞；`makePacket_03` 读 `msg[2..5]` 在短帧下读残留字节的越界隐患；DHT11 读取时关中断顶掉 PC 帧（改为总线静默才采样）。

### v1.0.0 · 2026-11-08 · 补齐 USART1 错误恢复 + 实测速率边界

- **改了什么**：`HAL_UART_ErrorCallback()` 由「只处理 USART2、其余直接 return」改为 USART1 / USART2 两路分别恢复，各自清 ORE、清对应计数器与判帧状态、重启 `HAL_UART_Receive_IT`；`sendMessage()` 与 USART1 回传的 `HAL_UART_Transmit()` 超时由写死的 100ms 改为 `(uint32_t)len * 2 + 10`；版本号统一为 `v1.0.0` 语义化三段式。
- **加了什么**：USART1 的 ORE 恢复通路；随帧长动态计算的发送超时；连续收发速率边界实测表（16ms 腰斩 / 100ms 完整）；「背靠背请求」「无请求—应答状态机」两条限制。
- **解决了什么**：USART1 一次 ORE 或线路噪声后永久收不到数据、只能复位的问题；9600 下超过 96 字节的帧被 HAL 判超时截断的问题；PC 高频连发时应答被腰斩却无文档解释的问题。

### v1.0 · 2026-11-07 · `95ceb87` · 快照：PC-Modbus 透传桥 v1 全链路跑通

- **改了什么**：工程角色从「Modbus 从机」切换为「双向透传桥」；串口接收回调独立到 `UART_Buf.c`，两路拆成独立缓冲与独立判帧状态；RS485 方向脚由 Damo 工程的 PB2 改为 PA1；回传 PC 由 ASCII 文本改为原始二进制帧。
- **加了什么**：USART1 接收链路（中断接收、5ms 判帧、PC 帧消费者）；两个消费者；`_write()` printf 重定向；`.ioc` 全局中断；本地 git 仓库与 GitHub 私有仓库首次快照。
- **解决了什么**：PC 请求帧进不了单片机、没有转发通路的问题；判帧完成标志未消费导致重复转发的问题；回传 ASCII 使主站无法解析且字节数膨胀 4.6 倍的问题。最终实测拿到变送器应答 `01 03 02 01 14 B9 DB`。

### v1.0 文档补充 · 2026-11-07

- **改了什么**：新增本 `README.md`（原仓库首页无任何说明）。
- **加了什么**：项目定位、硬件参数表、代码结构表、关键实现点、实测帧解析、已知限制、构建方式；并确立「每次推送三段式说明」的更新日志规范。
- **解决了什么**：`95ceb87` 快照内容无法自解释，clone 后无法判断这版代码在做什么、桥与从机的区别、以及遗留限制在哪里。
