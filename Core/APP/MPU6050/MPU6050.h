#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdint.h>
#include "i2c.h"

/* ================== MPU6050 寄存器地址 ================== */
#define MPU6050_WHO_AM_I       0x75
#define MPU6050_PWR_MGMT_1     0x6B
#define MPU6050_SMPLRT_DIV     0x19
#define MPU6050_ACCEL_CONFIG   0x1C
#define MPU6050_GYRO_CONFIG    0x1B
#define MPU6050_ACCEL_XOUT_H   0x3B   /* 从该地址起连续 14 字节：加速度(6) + 温度(2) + 角速度(6) */

/* ================== I2C 从机地址 ==================
 * 7 位地址：AD0 悬空/接地 -> 0x68，AD0 接 VCC -> 0x69。
 * 注意：HAL 的 HAL_I2C_Mem_Read/Write 传 7 位地址即可，
 * HAL 内部会自动左移并拼上 R/W 位。不要传 0xD0 这种已左移的 8 位地址。 */
#define MPU6050_ADDR           (0x68 << 1)

/* ================== 原始数据（未换算） ================== */
typedef struct
{
    int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
    int16_t Temp_RAW;   /* 原始温度码，换算 = /340.0 + 36.53 ℃ */
} MPU6050_RawData_t;

/* ================== 函数接口 ================== */

/* 初始化：唤醒设备 + 配采样率/量程。返回 0 成功，1 失败（WHO_AM_I 对不上）。 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);

/* 读 WHO_AM_I，正常应为 0x68。用于上板后第一件事——确认 I2C 通了没。 */
uint8_t MPU6050_GetID(I2C_HandleTypeDef *hi2c);

/* 一次读 14 字节原始数据（加速度 3 + 温度 1 + 角速度 3）。 */
void MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_RawData_t *raw);

#endif /* __MPU6050_H__ */
