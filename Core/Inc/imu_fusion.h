/* Верхний уровень инерциального модуля: калибровки, fusion 9 осей, азимут.
 * Платформо-независимый (без HAL).
 *
 * Вход: сырые данные датчиков в осях ПЛАТЫ.
 * Внутри: перестановка осей под корпус (см. imu_config.h), снятие смещений,
 * фильтр Madgwick, вычисление азимута с компенсацией наклона.
 */
#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include <stdbool.h>
#include <stdint.h>

#include "madgwick_ahrs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Калибровки (хранятся во flash, см. imu_flash.h). */
typedef struct {
    float gyro_bias[3];     /* рад/с */
    float mag_hard[3];      /* hard-iron, мкТл */
    float mag_scale[3];     /* soft-iron, безразмерные */
    float declination_deg;  /* магнитное склонение, + к востоку */
    float yaw_offset_deg;   /* смещение ZERO_YAW для fused-курса */
    float azimuth_offset_deg; /* смещение ZERO_YAW для азимута */
    uint8_t rate_hz;
    uint8_t mag_calibrated;
    uint8_t gyro_calibrated;
    uint8_t reserved;
} imu_calib_t;

/* Один отсчёт датчиков в осях платы. */
typedef struct {
    float ax, ay, az; /* м/с^2 */
    float gx, gy, gz; /* рад/с */
    float mx, my, mz; /* мкТл */
    bool mag_valid;   /* false, если магнитометр не отвечает/переполнен */
    float temp_c;
} imu_sample_t;

/* Результат fusion в осях корпуса (X вперёд, Y влево, Z вверх). */
typedef struct {
    float qw, qx, qy, qz; /* корпус->ENU */
    float roll_deg, pitch_deg, yaw_deg;
    float azimuth_deg; /* 0..360, геогр., tilt-comp */
    float wx, wy, wz;  /* рад/с, без дрейфа */
    float ax, ay, az;
    float mx, my, mz; /* калиброванные */
    bool fused_9x;
} imu_result_t;

/* Состояние сбора калибровки магнитометра (min/max по осям). */
typedef struct {
    uint8_t active;
    uint32_t count;
    uint32_t target;
    float min[3];
    float max[3];
} imu_mag_calib_t;

/* Состояние сбора смещения гироскопа. */
typedef struct {
    uint8_t active;
    uint32_t count;
    uint32_t target;
    double sum[3];
} imu_gyro_calib_t;

typedef struct {
    madgwick_t ahrs;
    imu_calib_t calib;
    imu_mag_calib_t mag_cal;
    imu_gyro_calib_t gyro_cal;
    float last_yaw_deg;
    float last_azimuth_deg;
    uint8_t seeded; /* 1 = кватернион инициализирован из аксель+маг */
} imu_fusion_t;

void imu_calib_defaults(imu_calib_t *c);

void imu_fusion_init(imu_fusion_t *f, const imu_calib_t *calib);
void imu_fusion_set_rate(imu_fusion_t *f, uint8_t rate_hz);

/* Попросить повторную инициализацию кватерниона по следующему хорошему
 * отсчёту (после ударов, долгой потери магнитометра и т.п.). */
void imu_fusion_request_reseed(imu_fusion_t *f);

/* Применить перестановку осей платы->корпус (на месте). */
void imu_apply_mount(float *x, float *y, float *z);

/* Главный шаг: калибровка входных данных + фильтр + углы. */
void imu_fusion_update(imu_fusion_t *f, const imu_sample_t *s, imu_result_t *r);

/* Азимут с компенсацией наклона векторным методом.
 * Вход - калиброванные векторы в осях корпуса. Возвращает 0..360. */
float imu_tilt_compensated_azimuth(float ax, float ay, float az,
                                   float mx, float my, float mz,
                                   float declination_deg);

/* --- Калибровка магнитометра (вращать плату во всех плоскостях) --- */
void imu_mag_calib_start(imu_fusion_t *f, uint32_t target_samples);
/* Скормить сырой вектор в осях КОРПУСА (mount применён, hard/soft сняты).
 * true = сбор завершён. */
bool imu_mag_calib_feed(imu_fusion_t *f, float mx, float my, float mz);
void imu_mag_calib_finish(imu_fusion_t *f, bool apply);
uint8_t imu_mag_calib_progress(const imu_fusion_t *f);

/* --- Калибровка гироскопа (плата неподвижна) --- */
void imu_gyro_calib_start(imu_fusion_t *f, uint32_t target_samples);
bool imu_gyro_calib_feed(imu_fusion_t *f, float gx, float gy, float gz);

/* Обнулить курс относительно текущего положения (mode=0) или снять (mode=1). */
void imu_zero_yaw(imu_fusion_t *f, uint8_t mode);

float imu_wrap180(float deg);
float imu_wrap360(float deg);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FUSION_H */
