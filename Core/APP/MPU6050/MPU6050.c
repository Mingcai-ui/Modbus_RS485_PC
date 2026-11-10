#include "mpu6050.h"

/* I2C 单次操作超时 (ms)。100 足够覆盖 100kHz 下读 14 字节的最坏情况。 */
#define MPU6050_I2C_TIMEOUT   100

/* MPU6050 初始化：返回 0 成功，1 失败。
 * 移植自电赛代码，只保留「唤醒 + 配置 + 读 ID」这条最朴素路径，
 * 砍掉了姿态解算 / 卡尔曼 / 互补滤波 / Yaw 积分——那些是电赛做平衡车
 * 用的，跑传输链路只需要原始数据。 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t data;

    /* 先读 WHO_AM_I，确认芯片在总线上、地址对。 */
    if (MPU6050_GetID(hi2c) != 0x68)
    {
        return 1;
    }

    /* PWR_MGMT_1 写 0：唤醒（复位后默认睡眠，bit6 SLEEP=1）。 */
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &data, 1, MPU6050_I2C_TIMEOUT);

    /* SMPLRT_DIV=7 -> 采样率 = 8kHz/(1+7) = 1kHz。 */
    data = 0x07;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_SMPLRT_DIV, 1, &data, 1, MPU6050_I2C_TIMEOUT);

    /* ACCEL_CONFIG：FS_SEL=0 -> 量程 ±2g。 */
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, &data, 1, MPU6050_I2C_TIMEOUT);

    /* GYRO_CONFIG：FS_SEL=0 -> 量程 ±250°/s。 */
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, &data, 1, MPU6050_I2C_TIMEOUT);

    return 0;
}

uint8_t MPU6050_GetID(I2C_HandleTypeDef *hi2c)
{
    uint8_t id = 0;
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_WHO_AM_I, 1, &id, 1, MPU6050_I2C_TIMEOUT);
    return id;
}

void MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_RawData_t *raw)
{
    uint8_t buf[14];

    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, buf, 14, MPU6050_I2C_TIMEOUT);

    /* 大端拼接：高字节在前。 */
    raw->Accel_X = (int16_t)(buf[0] << 8 | buf[1]);
    raw->Accel_Y = (int16_t)(buf[2] << 8 | buf[3]);
    raw->Accel_Z = (int16_t)(buf[4] << 8 | buf[5]);
    raw->Temp_RAW = (int16_t)(buf[6] << 8 | buf[7]);
    raw->Gyro_X = (int16_t)(buf[8] << 8 | buf[9]);
    raw->Gyro_Y = (int16_t)(buf[10] << 8 | buf[11]);
    raw->Gyro_Z = (int16_t)(buf[12] << 8 | buf[13]);
}
