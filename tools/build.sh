#!/usr/bin/env bash
# ============================================================
#  Modbus_RS485_PC 命令行编译验证脚本（不依赖 EIDE）
#  用途：重构过程中每个里程碑的"编译通过"验证
#  用法：bash tools/build.sh
#  说明：
#    - 使用 EIDE 自带的 arm-none-eabi-gcc（版本见下方 GCC 变量）
#    - 源文件清单 / 头文件路径 / 宏 / 链接脚本 与 .eide/eide.yml 保持一致
#    - 输出到 build/makecheck/，不干扰 EIDE 的 build/Debug/
# ============================================================
set -e

GCC="C:/Users/17107/.eide/tools/gcc_arm/bin/arm-none-eabi-gcc.exe"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/build/makecheck"
OBJ="$OUT/obj"

mkdir -p "$OBJ"
cd "$ROOT"

INCS=(
  -ICore/Inc
  -ICore/APP/Core
  -IDrivers/STM32F1xx_HAL_Driver/Inc
  -IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy
  -IDrivers/CMSIS/Device/ST/STM32F1xx/Include
  -IDrivers/CMSIS/Include
  -ICore/APP/Transport
  -ICore/APP/CRC
  -ICore/APP/Modbus
  -ICore/APP/DHT11
  -ICore/APP/OLED
  -ICore/APP/MPU6050
)
DEFS=(-DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB)
CFLAGS=(
  -mcpu=cortex-m3 -mthumb -Og -Wall -g
  -ffunction-sections -fdata-sections -funsigned-char
  --specs=nano.specs --specs=nosys.specs
)

SRCS=(
  Core/APP/Core/component.c
  Core/APP/Core/tick.c
  Core/APP/Core/critical.c
  Core/APP/CRC/CRC16.c
  Core/APP/DHT11/dht11.c
  Core/APP/MPU6050/mpu6050.c
  Core/APP/Modbus/Mb_port.c
  Core/APP/Modbus/modbus_rtu.c
  Core/APP/Transport/frame.c
  Core/APP/Transport/console.c
  Core/APP/OLED/OLED.c
  Core/APP/OLED/OLED_Data.c
  Core/Src/dma.c
  Core/Src/gpio.c
  Core/Src/i2c.c
  Core/Src/main.c
  Core/Src/stm32f1xx_hal_msp.c
  Core/Src/stm32f1xx_it.c
  Core/Src/syscalls.c
  Core/Src/sysmem.c
  Core/Src/system_stm32f1xx.c
  Core/Src/usart.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
)

OBJS=()
for src in "${SRCS[@]}"; do
  obj="$OBJ/$(basename "${src%.c}").o"
  "$GCC" -c -std=c11 "${INCS[@]}" "${DEFS[@]}" "${CFLAGS[@]}" -o "$obj" "$src"
  OBJS+=("$obj")
done

# 启动汇编文件（含预处理宏）
"$GCC" -c -x assembler-with-cpp "${DEFS[@]}" -mcpu=cortex-m3 -mthumb \
  -o "$OBJ/startup_stm32f103c8tx.o" Core/Startup/startup_stm32f103c8tx.s
OBJS+=("$OBJ/startup_stm32f103c8tx.o")

# 链接
"$GCC" "${CFLAGS[@]}" -T STM32F103C8TX_FLASH.ld -Wl,--gc-sections \
  -o "$OUT/Modbus_RS485_PC.elf" "${OBJS[@]}"

echo "=============================================="
echo "  BUILD OK: $OUT/Modbus_RS485_PC.elf"
echo "=============================================="
