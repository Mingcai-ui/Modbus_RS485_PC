# Modbus_RS485_PC — PC ↔ RS485 透传桥

> PC 向单片机发送 Modbus 请求帧，单片机**不进行帧校验**，只把这一帧原样搬上 RS485 总线，
> 替 PC 与温湿度变送器完成一次「请求—应答」交互（口头常说的"握手"），
> 再把变送器的应答帧原样回传 PC。

STM32F103C8T6 + HAL，裸机 `while(1)`，无 RTOS。当前版本 **v1.0**：全链路已跑通。

---

## 1. 这个版本做什么

单片机在此**不是 Modbus 从机，也不解析协议**，它是一条双向通道：

```
PC 主站 (串口助手 / Modbus Poll)
        │  USART1  115200 8N1   PA9(TX) / PA10(RX)
        ▼
┌───────────────────────────────────────────┐
│  STM32F103C8T6  透传桥                    │
│  PC_RX[] ──原样──> sendMessage() ──> USART2│
│  ModbusRx[] ──原样──> HAL_UART_Transmit ──>USART1
└───────────────────────────────────────────┘
        │  USART2  9600 8N1  PA2/PA3 + PA1(DE/RE)
        ▼
RS485 温湿度变送器（从机地址 0x01）
```

**一个容易混淆、但必须分清的区别：**

| | 做不做 | 由谁做 |
|---|---|---|
| **判帧**（这一帧什么时候结束） | ✅ 做 | 5ms 静默间隔，两路各一套 |
| **帧校验**（CRC / 从机地址 / 功能码） | ❌ 不做 | 交给 PC 端主站软件或变送器 |

也就是说：地址错的帧、CRC 错的帧、长度不完整的帧，单片机**一律照搬不过滤**。
这正是本工程要拿来验证「地址错 / CRC 错 / 采帧」三类用例的前提——脏帧能原样通过桥，才能在 PC 端观察真实行为。

> 严格讲 Modbus RTU 没有"握手"（不建立连接），这里是一次 `请求 → 应答` 事务；
> 本文档沿用口语说法，但代码实现的是原样转发，不做任何协商。

## 2. 硬件与参数

| 项 | 配置 |
|---|---|
| MCU | STM32F103C8T6，HSE 8MHz → PLL×9 = 72MHz |
| USART1（PC 侧） | 115200 / 8N1，PA9=TX、PA10=RX，中断收 + 阻塞发 |
| USART2（RS485 侧） | 9600 / 8N1，PA2=TX、PA3=RX，中断收 + 阻塞发 |
| RS485 方向脚 | **PA1**，`SET`=DE 有效(发送)、`RESET`=接收态；`gpio.c` 上电初始为接收态 |
| 判帧阈值 | `FRAME_GAP_MS = 5`（按 9600 侧 3.5T ≈ 3.65ms 定），SysTick 1ms 滴答 |
| 缓冲区 | `BUFFERSIZE = 255`，两路各自独立 |
| 从机 | 温湿度变送器，地址 0x01 |

## 3. 代码结构

| 文件 | 职责 |
|---|---|
| `Core/Src/main.c` | 初始化 + 置 PA1 为接收态 + 启动两路 `HAL_UART_Receive_IT`；`while(1)` 轮询两个消费者 |
| `Core/APP/UART_Buf/UART_Buf.c/.h` | 两路接收缓冲 `PC_RX[]` / `ModbusRx[]`、字节计数器、`HAL_UART_RxCpltCallback` 分流、`_write()` 把 printf 重定向到 USART1 |
| `Core/APP/Modbus/modbus_rtu.c/.h` | 两路 1ms 判帧 `uartTimer()` / `uartTimer_PC()`、两个消费者 `uartDataHandler_PC()` / `uartDataHandler_Modbus()`、`sendMessage()`（含 PA1 方向切换）、`printf_HXL()` 旁路日志 |
| `Core/Src/stm32f1xx_it.c` | `SysTick_Handler` 挂两个判帧函数；`USART1/2_IRQHandler` → `HAL_UART_IRQHandler` |
| `Core/APP/CRC/CRC16.c/.h` | Modbus CRC16 —— **本版本主链路不调用**，供从机遗留代码使用 |
| `.eide/eide.yml` / `Modbus_RS485_PC.ioc` | EIDE 编译配置 / CubeMX 引脚与 NVIC 配置 |

## 4. 关键实现点

- **两路完全隔离**：各有独立缓冲区、独立计数器（`DataCounter_PC` / `DataCounter_Modbus`）、独立判帧状态（`*_PC` 后缀）。共用一套计数器会让 PC 帧和变送器帧互相破坏判帧。
- **完成标志由消费者清**：`uartPacketComplatedFlag*` 在 `uartDataHandler_*()` 一进来就 RESET，同时清零计数器。若放在接收中断里清，或在消费者里不清，会导致同一帧被主循环重复转发刷屏，或下一帧首字节把未消费的标志抹掉。
- **先拷出再清空**：`memcpy` 到 `pcFrame` / `rsFrame` 后再 `memset` 接收区，避免发送期间被中断改写、以及短帧尾部残留旧数据。
- **方向脚只属于 RS485 那一路**：去往变送器走 `sendMessage()`（会切 PA1）；回传 PC 走 `HAL_UART_Transmit(&huart1, ...)`，**不碰 PA1**。
- **回传发原始二进制帧**：`HAL_UART_Transmit(&huart1, (uint8_t*)rsFrame, len, 100)`。PC 串口助手需切「HEX 显示」；若用 Modbus Poll 等主站软件，也必须收原始帧才解析得了。`printf_HXL()` 只作为旁路调试保留（它输出的是 ASCII 文本，7 字节会膨胀成 32 字节）。

## 5. 实测验证（v1.0）

```
PC  →  01 03 00 01 00 01 D5 CA     请求帧：地址01 / 03读保持寄存器 / 起始0x0001 / 读1个 / CRC D5 CA
PC  ←  01 03 02 01 14 B9 DB       应答帧：地址01 / 03 / 字节数02 / 数据0x0114(276) / CRC B9 DB
```

`0x0114 = 276`，若变送器按 ×0.1 标定即 27.6（℃/% 及倍率需查变送器手册确认）。
收到该帧即证明：`PC → USART1 → RS485(PA1) → 变送器 → USART2 → USART1 → PC` 全链路通。

## 6. 已知限制与待办

| 项 | 说明 |
|---|---|
| USART1 错误恢复 | `HAL_UART_ErrorCallback` 目前只处理 USART2；USART1 一次 ORE 会永久收不到数据，应照 USART2 补一段 |
| 从机遗留代码 | `uartDataHandler()` + `transmitDataMake()` + `makePacket_01/03/05/06/0F/10` + `sendException()` 主链路**均不调用**，是死代码，占 Flash，桥稳定后可删或另存分支 |
| 判帧阈值 | 5ms 按 9600 侧定；115200 侧 3.5T ≈ 0.3ms。PC 串口助手若把一帧拆成多次发且间隔 >5ms，会被切成多帧 —— 这正是「采帧」待测现象，不是 bug |
| 方向切换时机 | `sendMessage()` 在 `HAL_UART_Transmit` 返回后立刻拉低 PA1；严谨做法应显式等 `UART_FLAG_TC` 再切 |
| 注释陈旧 | `modbus_rtu.c` 文件头仍写「PB2 控制方向」，实际为 PA1，待改 |
| 未做压力测试 | 地址错 / CRC 错 / 分包采帧三类用例尚未系统验证 |

## 7. 构建

- **EIDE**（本项目实际使用）：打开 `Modbus_RS485_PC.code-workspace`，`.eide/eide.yml` 已配好三条头文件路径（`Core/APP/Modbus`、`Core/APP/UART_Buf`、`Core/APP/CRC`），arm-none-eabi-gcc 编译，产物在 `build/`（已忽略）。
- **STM32CubeIDE**：可直接导入 `.project` / `.cproject`。修改引脚后重新生成代码时，注意 `stm32f1xx_it.c` 的 `USER CODE` 区与 NVIC 勾选不要被覆盖。
- 仓库已忽略 `/build`、`/bin`、`/obj`、`/out`、`/.settings` 等产物。

---

## 更新日志

> **规范：每次更新推送，都必须说明三段 —— 改了什么 / 加了什么 / 解决了什么。**
> `git commit` 的 message、推送时的说明、以及此处的每一条记录，都按这个结构写。

### v1.0 · 2026-11-07 · `95ceb87` · 快照：PC-Modbus 透传桥 v1 全链路跑通

- **改了什么**：工程角色从「Modbus 从机」切换为「双向透传桥」；串口接收回调独立到 `UART_Buf.c`，两路拆成独立缓冲与独立判帧状态；RS485 方向脚由 Damo 工程的 PB2 改为 PA1；回传 PC 由 `printf_HXL()` 的 ASCII 文本改为 `HAL_UART_Transmit(&huart1, ...)` 原始二进制帧。
- **加了什么**：USART1 接收链路（中断接收、5ms 判帧、PC 帧消费者）；`uartDataHandler_PC()` / `uartDataHandler_Modbus()` 两个消费者；`_write()` printf 重定向；`.ioc` 中 USART1/2 全局中断与 `stm32f1xx_it.c` 两个 IRQHandler；本地 git 仓库与 GitHub 私有仓库首次快照。
- **解决了什么**：PC 请求帧原先根本进不了单片机（USART1 只发不收）、进了也没有转发通路的问题；判帧完成标志未消费导致主循环重复转发、计数器不清零导致后续帧追加错乱的问题；回传 ASCII 文本使主站软件无法解析、且字节数膨胀 4.6 倍的问题。最终实测拿到变送器应答 `01 03 02 01 14 B9 DB`，端到端链路打通。

### v1.0 文档补充 · 2026-11-07

- **改了什么**：新增本 `README.md`（原仓库首页无任何说明）。
- **加了什么**：项目定位、硬件参数表、代码结构表、关键实现点、实测帧解析、已知限制、构建方式；并确立「每次推送三段式说明」的更新日志规范。
- **解决了什么**：`95ceb87` 快照内容无法自解释，clone 后无法判断这版代码在做什么、桥与从机的区别、以及遗留限制在哪里。
