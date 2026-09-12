/* Фильтр ориентации Madgwick (2010), 9-осевой и 6-осевой режимы.
 * Платформо-независимый: только math.h.
 *
 * Система координат фильтра (NWU):
 *   X = север, Y = запад, Z = вверх.
 * Состояние q - кватернион поворота корпус->NWU (активный, v_nwu = q * v_body * q*).
 * Единичный кватернион = плата горизонтальна, ось X смотрит на север.
 *
 * Для ROS (ENU) кватернион пересчитывается: q_ros = qz(+90) * q.
 */
#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float q0, q1, q2, q3; /* кватернион корпус->NWU */
    float beta;           /* усиление коррекции (0.05..0.2) */
    float inv_sample_freq; /* 1 / частота, c */
} madgwick_t;

void madgwick_init(madgwick_t *f, float beta, float sample_freq_hz);
void madgwick_set_rate(madgwick_t *f, float sample_freq_hz);

/* Алгебраическая инициализация кватерниона из акселерометра (крен/тангаж)
 * и магнитометра (курс). ОБЯЗАТЕЛЬНА при старте: без неё фильтр сходится
 * с больших ошибок курса очень медленно (при 180 град. вообще застревает
 * в седловой точке в отсутствие шумов). Вход - векторы в осях корпуса. */
void madgwick_init_from_accel_mag(madgwick_t *f,
                                  float ax, float ay, float az,
                                  float mx, float my, float mz);

/* Полное обновление 9-осевым (гироскоп рад/с; аксель и магн. в любых единицах). */
void madgwick_update_9(madgwick_t *f,
                       float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float mx, float my, float mz);

/* Обновление 6-осевым (без магнитометра). Курс будет дрейфовать. */
void madgwick_update_6(madgwick_t *f,
                       float gx, float gy, float gz,
                       float ax, float ay, float az);

/* Углы ZYX в градусах: крен -180..180, тангаж -90..90, курс -180..180 (0=север). */
void madgwick_euler_nwu(const madgwick_t *f, float *roll_deg, float *pitch_deg, float *yaw_deg);

/* Кватернион корпус->ENU для ROS: q_ros = qz(+90град) * q. */
void madgwick_quat_ros_enu(const madgwick_t *f, float *qw, float *qx, float *qy, float *qz);

#ifdef __cplusplus
}
#endif

#endif /* MADGWICK_AHRS_H */
