/* Приложение инерциального модуля: опрос датчиков, fusion, UART-протокол.
 *
 * Использование в main.c:
 *   ImuApp_Init(&hi2c1, &huart2, &huart1);
 *   ImuApp_CommsStart();
 *   while (1) { ImuApp_Process(); }
 */
#ifndef IMU_APP_H
#define IMU_APP_H

#include "imu_fusion.h"
#include "stm32f3xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t frames_sent;
    uint32_t cmds_rx;
    uint32_t crc_errors; /* подсчёт в декодере не ведём, зарезервировано */
    uint32_t mpu_errors;
    uint32_t mag_errors;
} imu_app_stats_t;

void ImuApp_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart_data,
                 UART_HandleTypeDef *huart_dbg);
void ImuApp_CommsStart(void); /* NVIC + приём команд по прерываниям */
void ImuApp_Process(void);    /* вызывать в цикле как можно чаще */

const imu_fusion_t *ImuApp_GetFusion(void);
const imu_app_stats_t *ImuApp_GetStats(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_APP_H */
