/* Драйвер магнитометра QMC5883L по I2C через HAL.
 * Внимание: QMC5883L НЕ совместим по регистрам с HMC5883L!
 * Режим: continuous, 100 Гц ODR, диапазон 8 G, oversampling 512.
 */
#ifndef QMC5883L_H
#define QMC5883L_H

#include "stm32f3xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define QMC5883L_CHIP_ID_VAL 0xFFu

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t dev_addr; /* 8-битный адрес для HAL */
    float lsb_per_ut;  /* чувствительность */
    uint8_t ok;
} qmc5883l_t;

HAL_StatusTypeDef qmc5883l_init(qmc5883l_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7);

/* Чтение поля в мкТл. Возвращает HAL_OK, если есть свежие данные без переполнения. */
HAL_StatusTypeDef qmc5883l_read(qmc5883l_t *dev, float *mx, float *my, float *mz);

#ifdef __cplusplus
}
#endif

#endif /* QMC5883L_H */
