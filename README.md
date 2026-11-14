# Modbus_RS485_PC — PC ↔ RS485 路由器（01 透传 / 02 本机 DHT11 从机）

> 单片机不再是无脑透传桥，而是一个**带身份判断的路由器**：收到 PC 的 Modbus 请求帧后，
> 先验帧（长度 + CRC）、再看地址、最后决定这帧归谁处理。
>
> - 地址 `0x01` → 原样透传给 RS485 总线上的外部温湿度变送器，由它自己应答
> - 地址 `0x02` → 本机 DHT11 虚拟从机，读缓存组帧后从 USART1 直接回 PC
> - 其他地址 → 静默丢弃并计数
>
> STM32F103C8T6 + HAL，裸机 `while(1)`，无 RTOS。**当前版本 v1.0.0（main，纯透传桥）；
> 本分支 `feat/modbus-router` 为 v2.0.0 候选**，新增路由与 02 从机，待上板实测后合并 main 并打 tag。

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
│  DHT11(PA11) ── 主循环 2s 采样 ──> ModbusRegister[0..1]   │
└───────────────────────────────────────────────────────────┘
        │  USART2  9600 8N1  PA2/PA3 + PA1(DE/RE)
        ▼
RS485 温湿度变送器（从机地址 0x01）
```

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
| `Core/Src/main.c` | 初始化 + `mb_port_init()` + 注册 `frame_tick` + `frame_init()` 启动两路接收 + `DHT11_Init`；`while(1)` 轮询两个消费者 + 周期采样 DHT11 + OLED 刷屏 |
| `Core/APP/Transport/frame.c/.h` | 两路接收缓冲 + 判帧状态机 + `HAL_UART_RxCpltCallback` / `HAL_UART_ErrorCallback`；通道用 `frame_slot_t channels[]` 数组 + `frame_ch_t` 枚举，取代「变量名加 `_PC` 后缀」的复制粘贴；上层只通过 `frame_poll()` 取帧 |
| `Core/APP/Transport/console.c/.h` | `_write()` printf 重定向到 USART3（强符号覆盖 `syscalls.c` 的 weak 版本，不改 CubeMX 生成文件） |
| `Core/APP/Core/tick.c/.h` | 1ms tick 分发器（已启用）：`tick_register()` 注册回调，`SysTick_Handler` 只调 `tick_handle()`；中断层不再认识协议层 |
| `Core/APP/Core/component.c/.h`、`critical.c/.h` | 组件注册表、可重入临界区 —— M3/M4 预留骨架，当前未被调用 |
| `Core/APP/Modbus/modbus_rtu.c/.h` | 消费者 `uartDataHandler_PC()` / `uartDataHandler_Modbus()`、`checkFrame()` 判决、`sendMessage()`（按地址选出口）、`transmitDataMake()` + `makePacket_*` 功能码分发、`sendException()` 异常响应、四类丢弃计数（判帧已下沉到 `Transport/frame`） |
| `Core/APP/CRC/CRC16.c/.h` | Modbus CRC16 —— `checkFrame()` 与 `makePacket_*` 都调用，多项式 0xA001 反射、初值 0xFFFF |
| `Core/APP/DHT11/dht11.c/.h` | DHT11 单总线驱动（非阻塞调度 + 关中断读取），见 `DHT11/DHT11_介绍.md` |
| `Core/Src/stm32f1xx_it.c` | `SysTick_Handler` 调 `tick_handle()` 统一分发（不再直接认识 Modbus）；`USART1/2_IRQHandler` → `HAL_UART_IRQHandler` |
| `tools/build.sh` | 独立命令行编译验证脚本：用 EIDE 自带 GCC 输出到 `build/makecheck/`，不干扰 EIDE 的 `build/Debug` |
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

### 5.2 地址 02 本机 DHT11（待上板实测）

| 请求帧 | 应答帧 | 含义 |
|---|---|---|
| `02 03 00 00 00 02 C4 38` | `02 03 04 01 13 02 4E B8 5E` | 温度 0x0113=275 → 27.5℃；湿度 0x024E=590 → 59.0%RH |
| `02 03 00 09 00 04 <CRC>` | `02 83 02 30 F1` | 起始 9 + 4 > 10，越界 → 异常 0x02 |
| `02 06 00 00 00 64 <CRC>` | `02 86 01 <CRC>` | 02 只读，写请求 → 异常 0x01 |

> 寄存器布局：`0`=温度×10 的 int16 补码，`1`=湿度×10 的 uint16。温湿度变送器行业通用布局，
> 换 DHT22 / SHT30 也不用改 PC 端。零下温度注意 PC 端要用**有符号**显示，否则 −52 会显示成 65484。

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

- **EIDE**（本项目实际使用）：打开 `Modbus_RS485_PC.code-workspace`，`.eide/eide.yml` 已配好头文件路径（`Core/APP/{Core,Transport,Modbus,CRC,DHT11,OLED,MPU6050}`），arm-none-eabi-gcc 编译，产物在 `build/`（已忽略）。
- **命令行验证**：`bash tools/build.sh`，用 EIDE 自带的 GCC 输出到 `build/makecheck/`，不干扰 EIDE 的 `build/Debug`，适合每步改动后快速确认能编译。
- **STM32CubeIDE**：可直接导入 `.project` / `.cproject`。修改引脚后重新生成代码时，注意 `stm32f1xx_it.c` 的 `USER CODE` 区与 NVIC 勾选不要被覆盖。
- 仓库已忽略 `/build`、`/bin`、`/obj`、`/out`、`/.settings` 等产物。

---

## 更新日志

> **规范：每次更新推送，都必须说明三段 —— 改了什么 / 加了什么 / 解决了什么。**
> `git commit` 的 message、推送时的说明、以及此处的每一条记录，都按这个结构写。

### v2.2.0 候选 · 2026-11-14 · `modbus_congji`（重构 M0–M2 + OLED 硬件 I2C）

- **改了什么**：`stm32f1xx_it.c` 的 `SysTick_Handler` 由直接调 `uartTimer()` / `uartTimer_PC()` 改为调 `tick_handle()`，`#include "modbus_rtu.h"` 改为 `"tick.h"`；`modbus_rtu.c/.h` 删除判帧三件套（`RxInterruptFlag` / `uartTimeCounter` / `uartPacketComplatedFlag` 及 `_PC` 版）、`uartTimer()` / `uartTimer_PC()` 与 `FRAME_GAP_MS`，两个消费者改用 `frame_poll()`；`main.c` 启动由两句 `HAL_UART_Receive_IT` 改为 `frame_init()`、tick 回调改注册 `frame_tick`；`_write()` 由 USART1 改为 USART3；OLED 由软件模拟 I2C 改回硬件 I2C1（PB8/PB9），引入 MPU6050 并增加 `MX_I2C1_Init()`；`.eide/eide.yml` 头文件路径去 `UART_Buf`、增 `Core/APP/Core` 与 `Core/APP/Transport`；删除 `Core/APP/UART_Buf/`。
- **加了什么**：`Core/APP/Core/{component,tick,critical}.c/.h` —— 组件注册表、1ms tick 分发器、可重入临界区三个可替换抽象（`tick` 已启用，`component`/`critical` 为 M3/M4 预留骨架，当前未调用）；`Core/APP/Transport/frame.c/.h` —— 两路接收缓冲 + 判帧状态机 + 两个 HAL 回调，通道用 `frame_slot_t channels[]` 数组 + `frame_ch_t` 枚举取代复制粘贴；`Core/APP/Transport/console.c/.h` —— 日志通道；`tools/build.sh` —— 独立命令行编译脚本（EIDE 自带 GCC 14.3.1，输出 `build/makecheck/`，不干扰 `build/Debug`）。
- **解决了什么**：`SysTick` 中断文件反向依赖具体协议模块（换协议就要改中断文件）的问题；`UART_Buf.h` ↔ `modbus_rtu.h` 互相 include 的循环依赖；PC 侧接收上限被写死 8 字节、与 `bad_LONG`（>255）语义冲突的问题（统一为 255）；`printf` 与 Modbus 帧流共用 USART1、ASCII 日志插进二进制帧导致 PC 端乱码的问题（切 USART3）；判帧逻辑两份复制粘贴、加第三路就要再抄一遍的问题（合并为一个 `for` 循环）。**并定位了 16ms 压测丢包从 10% 劣化到 99% 的根因**：`OLED_Update()` 每轮阻塞刷 8 页（约 93ms @ I2C 100kHz），主循环周期被拉到约 100ms，PC 的 6 帧在缓冲里粘连成 48 字节 → CRC 全错整批丢弃；协议路径本身无问题（把 while 内 OLED 调用注释后 0 丢包）。
- **已知未修（下一轮处理）**：`main.c:137` 的 `(uint16_t)DHT11_GetTemp(&dht) * 10.0f` 仍是「先截断再乘」（B1，25.6℃ 会记成 250）；`DHT11_LOG_ENABLE` 已沦为无任何 `#if` 引用的死宏，改它不会打印；OLED 每轮全屏刷新未降频（需 tick 节流 + `OLED_UpdateArea` + I2C 400kHz 三杠杆并行）；`FRAME_GAP_MS = 5` 决定吞吐上限约 175 帧/秒，高吞吐从未被真正验证（PC 侧 USB-串口 Latency Timer 默认 16ms，标称「1ms 间隔」在线上实际仍是约 16ms）。

### v2.1.0 候选 · 2026-11-12 · `feat/slave02-dht11`（本分支，未合并 main / 未打 tag）

- **改了什么**：`sendMessage()`、`reply_port()`、`uartDataHandler_Modbus()` 三处发送出口由直接调 `HAL_UART_Transmit` 改为统一走 `mb_port_send()`；`main.c` 的 RS485 方向脚初始化由 `HAL_GPIO_WritePin` 改为 `mb_port_init()`；`dht11.c` 由「裸函数 + 全局变量」改为句柄式 API（`DHT11_Handle_t` + `Init/Handler/GetTemp/GetHumi`），并补全逐行注释（仅注释，逻辑未变）；CubeMX 增加 USART3（PB10=TX / PB11=RX，115200 8N1）及其中断。
- **加了什么**：`Core/APP/Modbus/Mb_port.c/.h` 端口抽象层——链路层只认 `MB_PORT_PC` / `MB_PORT_RS485` 两个编号，DE 拉高拉低与「发送超时随帧长走」都收进端口层；`main.c` 主循环恢复 DHT11 周期采样（2s），成功时写 `ModbusRegister[0..1]`，并在 OLED 上显示温度/湿度原始值；USART3 初始化、MSP 配置与 `USART3_IRQHandler`；`/.workbuddy/` 加入 `.gitignore`（工作区记忆与参考实现不入库）；主站 P0 参考实现（表驱动轮询 + 接收分流 + PC 帧排队，见 `.workbuddy/ref/master-p0/`，未纳入版本库）。
- **解决了什么**：RS485 方向脚散落在 `main.c` 与 `modbus_rtu.c` 两处、改一处忘一处的隐患；OLED 改造时 DHT11 采样块被整块删除、导致 0x02 地址寄存器恒 0 的问题；为主站日志腾出独立串口通道（USART3），避免 `printf` 字节插进 PC 的 Modbus 帧流把应答打断。
- **已知未修（下一轮处理）**：`main.c` 的 `(uint16_t)DHT11_GetTemp(&dht) * 10.0f` 是「先截断再乘」，27.5℃ 会记成 270 而非 275；`UART_Buf.c` 把 PC 侧接收上限由 `BUFFERSIZE` 改成 `8`，与 `bad_LONG`（>255）语义冲突，将来开放 0x06/0x10 写功能码时合法长帧会被误判；`_write()` 仍重定向到 `huart1`，日志尚未真正切到 USART3。

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
