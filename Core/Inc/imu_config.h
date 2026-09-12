/* Центральный файл конфигурации инерционного модуля.
 * Плата: STM32F303 + MPU6050 (акселерометр/гироскоп) + QMC5883L (магнитометр).
 * Связь с роботом: USART2, 115200, бинарный протокол (см. imu_protocol.h).
 * Отладка: USART1.
 */
#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Версия прошивки ---------- */
#define IMU_FW_VERSION_MAJOR 0
#define IMU_FW_VERSION_MINOR 1
#define IMU_FW_VERSION_PATCH 0
#define IMU_BOARD_NAME "STM32F303-IMU"

/* ---------- Частота выдачи ориентации, Гц (10/25/50/100) ---------- */
#define IMU_DEFAULT_RATE_HZ 50u

/* ---------- UART ---------- */
#define IMU_UART_BAUD_DATA  115200u /* USART2 -> робот (ROS 2) */
#define IMU_UART_BAUD_DEBUG 9600u   /* USART1 -> консоль отладки */

/* ---------- I2C адреса (7-битные) ---------- */
#define IMU_MPU6050_ADDR7 0x68u /* AD0 притянут к GND */
#define IMU_QMC5883L_ADDR7 0x0Du

/* ---------- Диапазоны MPU6050 ---------- */
#define IMU_MPU_ACCEL_FS_G  4   /* 2/4/8/16 g */
#define IMU_MPU_GYRO_FS_DPS 500 /* 250/500/1000/2000 град/с */

/* ---------- Фильтр Madgwick ---------- */
#define IMU_MADGWICK_BETA 0.10f /* 0.05..0.2: больше = быстрее сходимость, больше шум */

/* ---------- Магнитное склонение по умолчанию, град (+ к востоку).
 * Москва ~ +11.5. Точное значение для своей точки смотрите на
 * magnetic-declination.com и задавайте командой SET_DECLINATION. */
#define IMU_DECLINATION_DEG_DEFAULT 11.5f

/* ---------- Монтаж платы.
 * Оси корпуса датчика должны совпадать с осями робота:
 *   X вперёд, Y влево, Z вверх (REP-103).
 * Если плата установлена иначе, задайте перестановку/знаки ниже.
 * Формат: какая ось датчика идёт на каждую ось корпуса + знак.
 * Пример: корпус X = +датчик Y  => MOUNT_X_SRC=1, MOUNT_X_SIGN=+1. */
#define IMU_MOUNT_X_SRC 0
#define IMU_MOUNT_Y_SRC 1
#define IMU_MOUNT_Z_SRC 2
#define IMU_MOUNT_X_SIGN (+1)
#define IMU_MOUNT_Y_SIGN (+1)
#define IMU_MOUNT_Z_SIGN (+1)

/* ---------- Калибровки ---------- */
#define IMU_GYRO_CALIB_SAMPLES 100u /* при 50 Гц ~= 2 c неподвижности */
#define IMU_MAG_CALIB_SAMPLES 1500u /* при 50 Гц ~= 30 c вращений */
#define IMU_MAG_FIELD_MIN_UT 10.0f  /* правдоподобное поле Земли, мкТл */
#define IMU_MAG_FIELD_MAX_UT 120.0f

/* ---------- Flash для хранения калибровки (STM32F303, страница 2 КБ) ---------- */
#define IMU_FLASH_PAGE_ADDR ((uint32_t)0x0800F800u) /* стр. 31 при 64 КБ flash */
#define IMU_FLASH_MAGIC 0x494D5531u                 /* "IMU1" */

/* ---------- Таймауты HAL, мс ---------- */
#define IMU_I2C_TIMEOUT_MS 50u
#define IMU_UART_TX_TIMEOUT_MS 50u

#ifdef __cplusplus
}
#endif

#endif /* IMU_CONFIG_H */
