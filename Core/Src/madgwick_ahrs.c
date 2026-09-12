#include "madgwick_ahrs.h"

#include <math.h>

#ifndef IMU_PI
#define IMU_PI 3.14159265358979323846f
#endif
#define IMU_RAD2DEG (180.0f / IMU_PI)

void madgwick_init(madgwick_t *f, float beta, float sample_freq_hz)
{
    f->q0 = 1.0f;
    f->q1 = 0.0f;
    f->q2 = 0.0f;
    f->q3 = 0.0f;
    f->beta = beta;
    madgwick_set_rate(f, sample_freq_hz);
}

void madgwick_set_rate(madgwick_t *f, float sample_freq_hz)
{
    if (sample_freq_hz > 0.0f) {
        f->inv_sample_freq = 1.0f / sample_freq_hz;
    }
}

void madgwick_init_from_accel_mag(madgwick_t *f,
                                  float ax, float ay, float az,
                                  float mx, float my, float mz)
{
    float na = sqrtf(ax * ax + ay * ay + az * az);
    if (na < 1e-6f) {
        return; /* нет данных - оставляем как было */
    }
    /* Крен/тангаж из акселерометра (согласованы с madgwick_euler_nwu:
     * крен + при крене вправо, тангаж + при наклоне носом вниз). */
    float roll = atan2f(ay, az);
    float pitch = atan2f(ax, sqrtf(ay * ay + az * az));

    /* Горизонтируем магнитное поле: m_lvl = Ry(pitch) * Rx(roll) * m */
    float sr = sinf(roll), cr = cosf(roll);
    float sp = sinf(pitch), cp = cosf(pitch);
    float m1x = mx;
    float m1y = cr * my - sr * mz;
    float m1z = sr * my + cr * mz;
    float m2x = cp * m1x + sp * m1z;
    float m2y = m1y;

    float yaw = 0.0f;
    if ((m2x * m2x + m2y * m2y) > 1e-12f) {
        yaw = atan2f(-m2y, m2x);
    }

    /* Кватернион ZYX: q = qz(yaw) * qy(pitch) * qx(roll) */
    float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
    float cpr = cosf(pitch * 0.5f), spr = sinf(pitch * 0.5f);
    float crr = cosf(roll * 0.5f), srr = sinf(roll * 0.5f);
    f->q0 = crr * cpr * cy + srr * spr * sy;
    f->q1 = srr * cpr * cy - crr * spr * sy;
    f->q2 = crr * spr * cy + srr * cpr * sy;
    f->q3 = crr * cpr * sy - srr * spr * cy;
}

/* Быстрый обратный корень. На F303 с FPU обычный 1/sqrtf достаточно быстр. */
static float inv_sqrt(float x)
{
    return 1.0f / sqrtf(x);
}

void madgwick_update_9(madgwick_t *f,
                       float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float mx, float my, float mz)
{
    float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;

    /* Нормируем акселерометр и магнитометр */
    float norm = ax * ax + ay * ay + az * az;
    if (norm < 1e-12f) {
        return;
    }
    norm = inv_sqrt(norm);
    ax *= norm;
    ay *= norm;
    az *= norm;

    norm = mx * mx + my * my + mz * mz;
    if (norm < 1e-12f) {
        /* Магнитометр недоступен - откат на 6 осей */
        madgwick_update_6(f, gx, gy, gz, ax, ay, az);
        return;
    }
    norm = inv_sqrt(norm);
    mx *= norm;
    my *= norm;
    mz *= norm;

    /* Вспомогательные произведения */
    float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
    float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
    float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;

    /* Опорное направление поля Земли: h = q* x m (магн. в системе NWU) */
    float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    float hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    float hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
    float bx = sqrtf(hx * hx + hy * hy);
    float bz = hz;

    /* Градиентный спуск: направление ошибки */
    float halfvx = q1q3 - q0q2;
    float halfvy = q0q1 + q2q3;
    float halfvz = q0q0 - 0.5f + q3q3;
    float halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    float halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    float halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    float halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    float halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    float halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    /* Коррекция гироскопа */
    gx += f->beta * halfex * 2.0f;
    gy += f->beta * halfey * 2.0f;
    gz += f->beta * halfez * 2.0f;

    /* Интегрирование: qDot = 0.5 * q x omega */
    gx *= 0.5f * f->inv_sample_freq;
    gy *= 0.5f * f->inv_sample_freq;
    gz *= 0.5f * f->inv_sample_freq;
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    /* Нормализация */
    norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    f->q0 = q0 * norm;
    f->q1 = q1 * norm;
    f->q2 = q2 * norm;
    f->q3 = q3 * norm;
}

void madgwick_update_6(madgwick_t *f,
                       float gx, float gy, float gz,
                       float ax, float ay, float az)
{
    float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;

    float norm = ax * ax + ay * ay + az * az;
    if (norm < 1e-12f) {
        return;
    }
    norm = inv_sqrt(norm);
    ax *= norm;
    ay *= norm;
    az *= norm;

    float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3, q3q3 = q3 * q3;

    float halfvx = q1q3 - q0q2;
    float halfvy = q0q1 + q2q3;
    float halfvz = q0q0 - 0.5f + q3q3;

    float halfex = ay * halfvz - az * halfvy;
    float halfey = az * halfvx - ax * halfvz;
    float halfez = ax * halfvy - ay * halfvx;

    gx += f->beta * halfex * 2.0f;
    gy += f->beta * halfey * 2.0f;
    gz += f->beta * halfez * 2.0f;

    gx *= 0.5f * f->inv_sample_freq;
    gy *= 0.5f * f->inv_sample_freq;
    gz *= 0.5f * f->inv_sample_freq;
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    f->q0 = q0 * norm;
    f->q1 = q1 * norm;
    f->q2 = q2 * norm;
    f->q3 = q3 * norm;
}

void madgwick_euler_nwu(const madgwick_t *f, float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;
    float roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp > 1.0f) {
        sinp = 1.0f;
    } else if (sinp < -1.0f) {
        sinp = -1.0f;
    }
    float pitch = asinf(sinp);
    float yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
    *roll_deg = roll * IMU_RAD2DEG;
    *pitch_deg = pitch * IMU_RAD2DEG;
    *yaw_deg = yaw * IMU_RAD2DEG;
}

void madgwick_quat_ros_enu(const madgwick_t *f, float *qw, float *qx, float *qy, float *qz)
{
    /* q_ros = qz(+90град) x q, где qz = (cos45, 0, 0, sin45).
     * Умножение кватернионов: (a0 + a)(b0 + b). */
    const float c = 0.7071067811865475f; /* cos45 = sin45 */
    float b0 = f->q0, b1 = f->q1, b2 = f->q2, b3 = f->q3;
    *qw = c * b0 - c * b3;
    *qx = c * b1 - c * b2;
    *qy = c * b1 + c * b2;
    *qz = c * b0 + c * b3;
}
