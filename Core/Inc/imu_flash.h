/* Хранение калибровки во flash STM32F303 (одна страница 2 КБ).
 * Формат: magic "IMU1" + версия + структура + CRC16.
 */
#ifndef IMU_FLASH_H
#define IMU_FLASH_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_fusion.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Загрузить калибровку. false = пусто/битая (используйте defaults). */
bool imu_flash_load(imu_calib_t *calib);

/* Сохранить калибровку (стирает страницу целиком). */
bool imu_flash_save(const imu_calib_t *calib);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FLASH_H */
