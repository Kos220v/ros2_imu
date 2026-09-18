/* Драйвер магнитометров семейства 5883L по I2C через HAL.
 *
 * Поддерживаемые варианты (автоопределение):
 *  - QMC5883L (QST; на модулях GY-271/GY-273 маркируется как QMC5883L /
 *    HA5883 / DB5883 — «5035» и похожие во второй строке маркировки —
 *    код партии): I2C 7-битный 0x0D, CHIP_ID (рег. 0x0D) = 0xFF,
 *    данные 0x00-0x05 (LSB first, порядок X,Y,Z), STATUS 0x06,
 *    CTRL1 0x09, CTRL2 0x0A, SET/RESET 0x0B. (datasheet QST rev 1.0)
 *  - HMC5883L (Honeywell; классические GY-271/GY-273): I2C 7-битный
 *    0x1E, ID-регистры 0x0A-0x0C = "H43", данные 0x03-0x08 (MSB first,
 *    порядок X,Z,Y), CRA 0x00, CRB 0x01, MODE 0x02.
 *
 * Важно: варианты НЕ совместимы по регистрам!
 * Режим: continuous, QMC 100 Гц (HMC 75 Гц — максимум), диапазон ±8 G.
 */
#ifndef QMC5883L_H
#define QMC5883L_H

#include "stm32f3xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define QMC5883L_ADDR7 0x0Du
#define HMC5883L_ADDR7 0x1Eu

typedef enum {
    MAG5883_QMC = 0, /* QMC5883L / HA5883 / DB5883 */
    MAG5883_HMC,     /* HMC5883L (Honeywell) */
} mag5883_type_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t dev_addr; /* 8-битный адрес для HAL */
    mag5883_type_t type;
    float lsb_per_ut;  /* чувствительность, LSB на мкТл */
    uint8_t ok;
} qmc5883l_t;

/* Инициализация: определяет вариант на указанном адресе, если там
 * датчика нет — пробует другой стандартный адрес (0x0D / 0x1E). */
HAL_StatusTypeDef qmc5883l_init(qmc5883l_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7);

/* Чтение поля в мкТл. Возвращает HAL_OK, если есть свежие данные без переполнения. */
HAL_StatusTypeDef qmc5883l_read(qmc5883l_t *dev, float *mx, float *my, float *mz);

#ifdef __cplusplus
}
#endif

#endif /* QMC5883L_H */
