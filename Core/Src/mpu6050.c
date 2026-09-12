#include "mpu6050.h"

#include "imu_config.h"

/* Регистры MPU6050 */
#define MPU_REG_SMPLRT_DIV 0x19u
#define MPU_REG_CONFIG 0x1Au
#define MPU_REG_GYRO_CONFIG 0x1Bu
#define MPU_REG_ACCEL_CONFIG 0x1Cu
#define MPU_REG_INT_PIN_CFG 0x37u
#define MPU_REG_ACCEL_XOUT_H 0x3Bu
#define MPU_REG_PWR_MGMT_1 0x6Bu
#define MPU_REG_WHO_AM_I 0x75u

#define IMU_M_PI 3.14159265358979323846f
#define IMU_G0 9.80665f

static HAL_StatusTypeDef mpu_write(mpu6050_t *dev, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(dev->hi2c, dev->dev_addr, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, IMU_I2C_TIMEOUT_MS);
}

static float mpu_acc_lsb(uint8_t fs_g)
{
    switch (fs_g) {
    case 2:
        return 16384.0f;
    case 4:
        return 8192.0f;
    case 8:
        return 4096.0f;
    default:
        return 2048.0f; /* 16 g */
    }
}

static float mpu_gyro_lsb(uint16_t fs_dps)
{
    switch (fs_dps) {
    case 250:
        return 131.0f;
    case 500:
        return 65.5f;
    case 1000:
        return 32.8f;
    default:
        return 16.4f; /* 2000 */
    }
}

static uint8_t mpu_acc_cfg(uint8_t fs_g)
{
    switch (fs_g) {
    case 2:
        return 0x00u;
    case 4:
        return 0x08u;
    case 8:
        return 0x10u;
    default:
        return 0x18u;
    }
}

static uint8_t mpu_gyro_cfg(uint16_t fs_dps)
{
    switch (fs_dps) {
    case 250:
        return 0x00u;
    case 500:
        return 0x08u;
    case 1000:
        return 0x10u;
    default:
        return 0x18u;
    }
}

HAL_StatusTypeDef mpu6050_init(mpu6050_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    dev->hi2c = hi2c;
    dev->dev_addr = (uint16_t)((uint16_t)addr7 << 1);
    dev->acc_lsb_per_g = mpu_acc_lsb(IMU_MPU_ACCEL_FS_G);
    dev->gyro_lsb_per_dps = mpu_gyro_lsb(IMU_MPU_GYRO_FS_DPS);
    dev->ok = 0;
    dev->who_id = 0;

    if (HAL_I2C_IsDeviceReady(hi2c, dev->dev_addr, 3, IMU_I2C_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }

    uint8_t who = 0;
    if (HAL_I2C_Mem_Read(hi2c, dev->dev_addr, MPU_REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT,
                         &who, 1, IMU_I2C_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }
    /* 0x68 = MPU6050; 0x70 = MPU6500/клон (GY-521), совместим по регистрам */
    if (who != MPU6050_WHO_AM_I_VAL && who != MPU6500_WHO_AM_I_VAL) {
        return HAL_ERROR;
    }
    dev->who_id = who;

    /* Пробуждение: источник тактирования - PLL по гиро X */
    if (mpu_write(dev, MPU_REG_PWR_MGMT_1, 0x01u) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(10);
    /* Sample rate: 1 кГц / (1 + 9) = 100 Гц внутренних */
    if (mpu_write(dev, MPU_REG_SMPLRT_DIV, 9u) != HAL_OK) {
        return HAL_ERROR;
    }
    /* DLPF ~42 Гц */
    if (mpu_write(dev, MPU_REG_CONFIG, 0x03u) != HAL_OK) {
        return HAL_ERROR;
    }
    if (mpu_write(dev, MPU_REG_GYRO_CONFIG, mpu_gyro_cfg(IMU_MPU_GYRO_FS_DPS)) != HAL_OK) {
        return HAL_ERROR;
    }
    if (mpu_write(dev, MPU_REG_ACCEL_CONFIG, mpu_acc_cfg(IMU_MPU_ACCEL_FS_G)) != HAL_OK) {
        return HAL_ERROR;
    }
    /* BYPASS_EN: магнитометр на AUX-линиях виден с общей шины I2C */
    if (mpu_write(dev, MPU_REG_INT_PIN_CFG, 0x02u) != HAL_OK) {
        return HAL_ERROR;
    }

    dev->ok = 1;
    return HAL_OK;
}

HAL_StatusTypeDef mpu6050_read(mpu6050_t *dev,
                               float *ax, float *ay, float *az,
                               float *gx, float *gy, float *gz,
                               float *temp_c)
{
    uint8_t raw[14];
    if (HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr, MPU_REG_ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw),
                         IMU_I2C_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }

    int16_t ax_r = (int16_t)((uint16_t)((uint16_t)raw[0] << 8) | raw[1]);
    int16_t ay_r = (int16_t)((uint16_t)((uint16_t)raw[2] << 8) | raw[3]);
    int16_t az_r = (int16_t)((uint16_t)((uint16_t)raw[4] << 8) | raw[5]);
    int16_t t_r = (int16_t)((uint16_t)((uint16_t)raw[6] << 8) | raw[7]);
    int16_t gx_r = (int16_t)((uint16_t)((uint16_t)raw[8] << 8) | raw[9]);
    int16_t gy_r = (int16_t)((uint16_t)((uint16_t)raw[10] << 8) | raw[11]);
    int16_t gz_r = (int16_t)((uint16_t)((uint16_t)raw[12] << 8) | raw[13]);

    const float a_k = IMU_G0 / dev->acc_lsb_per_g;
    const float g_k = (IMU_M_PI / 180.0f) / dev->gyro_lsb_per_dps;
    *ax = (float)ax_r * a_k;
    *ay = (float)ay_r * a_k;
    *az = (float)az_r * a_k;
    *gx = (float)gx_r * g_k;
    *gy = (float)gy_r * g_k;
    *gz = (float)gz_r * g_k;
    *temp_c = (float)t_r / 340.0f + 36.53f;
    return HAL_OK;
}
