#include "imu_fusion.h"

#include <math.h>

#include "imu_config.h"

#ifndef IMU_PI
#define IMU_PI 3.14159265358979323846f
#endif

float imu_wrap180(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg <= -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

float imu_wrap360(float deg)
{
    while (deg >= 360.0f) {
        deg -= 360.0f;
    }
    while (deg < 0.0f) {
        deg += 360.0f;
    }
    return deg;
}

void imu_calib_defaults(imu_calib_t *c)
{
    c->gyro_bias[0] = 0.0f;
    c->gyro_bias[1] = 0.0f;
    c->gyro_bias[2] = 0.0f;
    c->mag_hard[0] = 0.0f;
    c->mag_hard[1] = 0.0f;
    c->mag_hard[2] = 0.0f;
    c->mag_scale[0] = 1.0f;
    c->mag_scale[1] = 1.0f;
    c->mag_scale[2] = 1.0f;
    c->declination_deg = IMU_DECLINATION_DEG_DEFAULT;
    c->yaw_offset_deg = 0.0f;
    c->azimuth_offset_deg = 0.0f;
    c->rate_hz = IMU_DEFAULT_RATE_HZ;
    c->mag_calibrated = 0;
    c->gyro_calibrated = 0;
    c->reserved = 0;
}

void imu_fusion_init(imu_fusion_t *f, const imu_calib_t *calib)
{
    f->calib = *calib;
    madgwick_init(&f->ahrs, IMU_MADGWICK_BETA, (float)calib->rate_hz);
    f->mag_cal.active = 0;
    f->mag_cal.count = 0;
    f->mag_cal.target = 0;
    f->gyro_cal.active = 0;
    f->gyro_cal.count = 0;
    f->gyro_cal.target = 0;
    f->last_yaw_deg = 0.0f;
    f->last_azimuth_deg = 0.0f;
    f->seeded = 0;
}

void imu_fusion_request_reseed(imu_fusion_t *f)
{
    f->seeded = 0;
}

void imu_fusion_set_rate(imu_fusion_t *f, uint8_t rate_hz)
{
    f->calib.rate_hz = rate_hz;
    madgwick_set_rate(&f->ahrs, (float)rate_hz);
}

void imu_apply_mount(float *x, float *y, float *z)
{
    const float v[3] = {*x, *y, *z};
    const int sx = IMU_MOUNT_X_SRC, sy = IMU_MOUNT_Y_SRC, sz = IMU_MOUNT_Z_SRC;
    *x = (float)IMU_MOUNT_X_SIGN * v[sx];
    *y = (float)IMU_MOUNT_Y_SIGN * v[sy];
    *z = (float)IMU_MOUNT_Z_SIGN * v[sz];
}

/* Длина вектора. */
static float vec_norm(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

float imu_tilt_compensated_azimuth(float ax, float ay, float az,
                                   float mx, float my, float mz,
                                   float declination_deg)
{
    /* "Вверх" в корпусе = направление акселерометра (реакция опоры). */
    float na = vec_norm(ax, ay, az);
    if (na < 1e-6f) {
        return 0.0f;
    }
    float ux = ax / na, uy = ay / na, uz = az / na;

    /* Горизонтальная проекция поля -> направление на север. */
    float dot = mx * ux + my * uy + mz * uz;
    float nx = mx - dot * ux;
    float ny = my - dot * uy;
    float nz = mz - dot * uz;
    float nn = vec_norm(nx, ny, nz);
    if (nn < 1e-6f) {
        return 0.0f;
    }
    nx /= nn;
    ny /= nn;
    nz /= nn;

    /* Восток = север x верх. Нужна только X-компонента, т.к. азимут
     * считаем для оси X корпуса (1,0,0): X·восток = ex, X·север = nx. */
    float ex = ny * uz - nz * uy;

    /* Азимут оси X корпуса: atan2(восток, север), по часовой от севера. */
    float heading = atan2f(ex, nx) * (180.0f / IMU_PI);
    return imu_wrap360(heading + declination_deg);
}

void imu_fusion_update(imu_fusion_t *f, const imu_sample_t *s, imu_result_t *r)
{
    /* 1. Перестановка осей плата -> корпус. */
    float ax = s->ax, ay = s->ay, az = s->az;
    float gx = s->gx, gy = s->gy, gz = s->gz;
    float mx = s->mx, my = s->my, mz = s->mz;
    imu_apply_mount(&ax, &ay, &az);
    imu_apply_mount(&gx, &gy, &gz);
    imu_apply_mount(&mx, &my, &mz);

    /* 2. Снятие смещений. */
    gx -= f->calib.gyro_bias[0];
    gy -= f->calib.gyro_bias[1];
    gz -= f->calib.gyro_bias[2];
    mx = (mx - f->calib.mag_hard[0]) * f->calib.mag_scale[0];
    my = (my - f->calib.mag_hard[1]) * f->calib.mag_scale[1];
    mz = (mz - f->calib.mag_hard[2]) * f->calib.mag_scale[2];

    /* 3. Проверка правдоподобности мага (защита от железа рядом). */
    bool mag_ok = s->mag_valid;
    if (mag_ok) {
        float m = vec_norm(mx, my, mz);
        if (m < IMU_MAG_FIELD_MIN_UT || m > IMU_MAG_FIELD_MAX_UT) {
            mag_ok = false;
        }
    }

    /* 3.5. Первичная инициализация кватерниона из аксель+маг.
     * Без неё фильтр из покоя сходится с больших ошибок курса минутами. */
    if (!f->seeded && mag_ok) {
        float a = vec_norm(ax, ay, az);
        if (a > 5.0f && a < 20.0f) {
            madgwick_init_from_accel_mag(&f->ahrs, ax, ay, az, mx, my, mz);
            f->seeded = 1;
        }
    }

    /* 4. Фильтр. */
    if (mag_ok) {
        madgwick_update_9(&f->ahrs, gx, gy, gz, ax, ay, az, mx, my, mz);
    } else {
        madgwick_update_6(&f->ahrs, gx, gy, gz, ax, ay, az);
    }

    /* 5. Выходы. */
    madgwick_quat_ros_enu(&f->ahrs, &r->qw, &r->qx, &r->qy, &r->qz);
    madgwick_euler_nwu(&f->ahrs, &r->roll_deg, &r->pitch_deg, &r->yaw_deg);
    r->yaw_deg = imu_wrap180(r->yaw_deg - f->calib.yaw_offset_deg);

    if (mag_ok) {
        r->azimuth_deg = imu_tilt_compensated_azimuth(ax, ay, az, mx, my, mz,
                                                     f->calib.declination_deg);
        r->azimuth_deg = imu_wrap360(r->azimuth_deg - f->calib.azimuth_offset_deg);
        f->last_azimuth_deg = r->azimuth_deg;
    } else {
        /* Без мага азимут держим последним (6-осевой режим дрейфует). */
        r->azimuth_deg = f->last_azimuth_deg;
    }
    f->last_yaw_deg = r->yaw_deg;

    r->wx = gx;
    r->wy = gy;
    r->wz = gz;
    r->ax = ax;
    r->ay = ay;
    r->az = az;
    r->mx = mx;
    r->my = my;
    r->mz = mz;
    r->fused_9x = mag_ok;
}

/* ---------- Калибровка магнитометра ---------- */

void imu_mag_calib_start(imu_fusion_t *f, uint32_t target_samples)
{
    f->mag_cal.active = 1;
    f->mag_cal.count = 0;
    f->mag_cal.target = (target_samples > 0) ? target_samples : 1u;
    for (int i = 0; i < 3; i++) {
        f->mag_cal.min[i] = 1e9f;
        f->mag_cal.max[i] = -1e9f;
    }
}

bool imu_mag_calib_feed(imu_fusion_t *f, float mx, float my, float mz)
{
    if (!f->mag_cal.active) {
        return false;
    }
    const float v[3] = {mx, my, mz};
    for (int i = 0; i < 3; i++) {
        if (v[i] < f->mag_cal.min[i]) {
            f->mag_cal.min[i] = v[i];
        }
        if (v[i] > f->mag_cal.max[i]) {
            f->mag_cal.max[i] = v[i];
        }
    }
    f->mag_cal.count++;
    if (f->mag_cal.count >= f->mag_cal.target) {
        f->mag_cal.active = 0; /* сбор окончен, ждём решения применить/отменить */
        return true;
    }
    return false;
}

void imu_mag_calib_finish(imu_fusion_t *f, bool apply)
{
    f->mag_cal.active = 0;
    if (!apply || f->mag_cal.count == 0) {
        return;
    }
    float center[3], half[3];
    for (int i = 0; i < 3; i++) {
        center[i] = 0.5f * (f->mag_cal.max[i] + f->mag_cal.min[i]);
        half[i] = 0.5f * (f->mag_cal.max[i] - f->mag_cal.min[i]);
        if (half[i] < 1e-6f) {
            return; /* ось не прокачали - не применяем */
        }
    }
    float avg = (half[0] + half[1] + half[2]) / 3.0f;
    for (int i = 0; i < 3; i++) {
        f->calib.mag_hard[i] = center[i];
        f->calib.mag_scale[i] = avg / half[i];
    }
    f->calib.mag_calibrated = 1;
}

uint8_t imu_mag_calib_progress(const imu_fusion_t *f)
{
    if (f->mag_cal.target == 0) {
        return 0;
    }
    uint32_t p = (f->mag_cal.count * 100u) / f->mag_cal.target;
    return (p > 100u) ? 100u : (uint8_t)p;
}

/* ---------- Калибровка гироскопа ---------- */

void imu_gyro_calib_start(imu_fusion_t *f, uint32_t target_samples)
{
    f->gyro_cal.active = 1;
    f->gyro_cal.count = 0;
    f->gyro_cal.target = (target_samples > 0) ? target_samples : 1u;
    f->gyro_cal.sum[0] = 0.0;
    f->gyro_cal.sum[1] = 0.0;
    f->gyro_cal.sum[2] = 0.0;
}

bool imu_gyro_calib_feed(imu_fusion_t *f, float gx, float gy, float gz)
{
    if (!f->gyro_cal.active) {
        return false;
    }
    f->gyro_cal.sum[0] += (double)gx;
    f->gyro_cal.sum[1] += (double)gy;
    f->gyro_cal.sum[2] += (double)gz;
    f->gyro_cal.count++;
    if (f->gyro_cal.count >= f->gyro_cal.target) {
        double n = (double)f->gyro_cal.count;
        f->calib.gyro_bias[0] = (float)(f->gyro_cal.sum[0] / n);
        f->calib.gyro_bias[1] = (float)(f->gyro_cal.sum[1] / n);
        f->calib.gyro_bias[2] = (float)(f->gyro_cal.sum[2] / n);
        f->calib.gyro_calibrated = 1;
        f->gyro_cal.active = 0;
        return true;
    }
    return false;
}

/* ---------- ZERO_YAW ---------- */

void imu_zero_yaw(imu_fusion_t *f, uint8_t mode)
{
    if (mode == 0) {
        /* Запомнить текущие как ноль. Выход = raw - offset, поэтому новый
         * offset = raw = last + старый offset. */
        f->calib.yaw_offset_deg =
            imu_wrap180(f->last_yaw_deg + f->calib.yaw_offset_deg);
        f->calib.azimuth_offset_deg =
            imu_wrap360(f->last_azimuth_deg + f->calib.azimuth_offset_deg);
    } else {
        f->calib.yaw_offset_deg = 0.0f;
        f->calib.azimuth_offset_deg = 0.0f;
    }
}
