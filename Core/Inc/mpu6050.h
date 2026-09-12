/* Драйвер MPU6050 (акселерометр + гироскоп) по I2C через HAL.
 * Настройки диапазонов - в imu_config.h.
 */
#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f3xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_WHO_AM_I_VAL 0x68u
/* MPU6500 (и клоны на его основе, часто на модулях GY-521/GY-271): карта
 * регистров та же, что у MPU6050, различается только WHO_AM_I. */
#define MPU6500_WHO_AM_I_VAL 0x70u

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t dev_addr; /* 8-битный адрес для HAL */
    float acc_lsb_per_g;
    float gyro_lsb_per_dps;
    uint8_t ok;
    uint8_t who_id; /* WHO_AM_I: 0x68 = MPU6050, 0x70 = MPU6500/клон */
} mpu6050_t;

/* Инициализация: пробуждение, DLPF, диапазоны, bypass для AUX I2C. */
HAL_StatusTypeDef mpu6050_init(mpu6050_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7);

/* Чтение: аксель м/с^2, гиро рад/с, температура град.C. */
HAL_StatusTypeDef mpu6050_read(mpu6050_t *dev,
                               float *ax, float *ay, float *az,
                               float *gx, float *gy, float *gz,
                               float *temp_c);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
