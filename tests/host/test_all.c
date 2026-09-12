/* Хост-тесты инерциального модуля (запуск на ПК, без железа).
 * Сборка: tests/run_host_tests.sh
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "imu_app.h"
#include "imu_flash.h"
#include "imu_fusion.h"
#include "imu_protocol.h"
#include "madgwick_ahrs.h"
#include "mpu6050.h"
#include "qmc5883l.h"
#include "stm32f3xx_hal.h" /* стаб */

static int s_failures;
static int s_checks;

#define CHECK(cond, ...)                                                      \
    do {                                                                      \
        s_checks++;                                                           \
        if (!(cond)) {                                                        \
            s_failures++;                                                     \
            printf("FAIL %s:%d: ", __func__, __LINE__);                        \
            printf(__VA_ARGS__);                                              \
            printf("\n");                                                     \
        }                                                                     \
    } while (0)

#define CHECK_CLOSE(a, b, tol, fmt, ...)                                      \
    do {                                                                      \
        float _a = (float)(a), _b = (float)(b), _t = (float)(tol);            \
        CHECK(fabsf(_a - _b) <= _t, fmt " (got %.4f, want %.4f)",             \
              ##__VA_ARGS__, (double)_a, (double)_b);                          \
    } while (0)

#define T_PI 3.14159265358979323846f

/* Разность углов в градусах по модулю 360. */
static float ang_diff(float a, float b)
{
    float d = fmodf(a - b, 360.0f);
    if (d > 180.0f) {
        d -= 360.0f;
    }
    if (d < -180.0f) {
        d += 360.0f;
    }
    return fabsf(d);
}

/* Повернуть вектор (1,0,0) кватернионом. */
static void quat_rot_x(float qw, float qx, float qy, float qz, float out[3])
{
    out[0] = 1.0f - 2.0f * (qy * qy + qz * qz);
    out[1] = 2.0f * (qx * qy + qz * qw);
    out[2] = 2.0f * (qx * qz - qy * qw);
}

/* ---------- 1. CRC ---------- */

static void test_crc(void)
{
    const uint8_t v[] = "123456789";
    CHECK(imu_crc16_ccitt(v, 9) == 0x29B1u, "crc test vector");
    CHECK(imu_crc16_ccitt(NULL, 0) == 0xFFFFu, "crc empty");
}

/* ---------- 2. Протокол: roundtrip ---------- */

static void test_proto_roundtrip(void)
{
    imu_orientation_t m;
    memset(&m, 0, sizeof(m));
    m.ts_ms = 123456u;
    m.qw = 0.7071f;
    m.qx = -0.5f;
    m.qy = 0.5f;
    m.qz = 0.1f;
    m.roll_deg = -179.5f;
    m.pitch_deg = 45.25f;
    m.yaw_deg = 90.0f;
    m.azimuth_deg = 270.5f;
    m.wx = 0.01f;
    m.wy = -0.02f;
    m.wz = 1.5f;
    m.ax = -9.8f;
    m.ay = 0.1f;
    m.az = 0.2f;
    m.mx = 20.5f;
    m.my = -30.25f;
    m.mz = 48.0f;
    m.temp_c = 36.5f;
    m.status = 0x1Fu;
    m.calib_state = 1u;
    m.rate_hz = 50u;

    uint8_t payload[IMU_ORIENTATION_LEN];
    imu_orientation_encode(&m, payload);

    uint8_t frame[IMU_PROTO_MAX_FRAME];
    size_t n = imu_proto_encode(IMU_MSG_ORIENTATION, payload, IMU_ORIENTATION_LEN,
                                frame, sizeof(frame));
    CHECK(n == IMU_ORIENTATION_LEN + 6u, "frame len");
    CHECK(frame[0] == 0xAAu && frame[1] == 0x55u, "sync");
    CHECK(frame[2] == IMU_ORIENTATION_LEN + 1u, "len field");
    CHECK(frame[3] == IMU_MSG_ORIENTATION, "msg id");

    imu_decoder_t dec;
    imu_decoder_init(&dec);
    int got = 0;
    for (size_t i = 0; i < n; i++) {
        uint8_t id, len;
        const uint8_t *pl;
        if (imu_decoder_feed(&dec, frame[i], &id, &pl, &len)) {
            got++;
            CHECK(id == IMU_MSG_ORIENTATION, "decoded id");
            CHECK(len == IMU_ORIENTATION_LEN, "decoded len");
            imu_orientation_t m2;
            CHECK(imu_orientation_decode(pl, len, &m2), "orientation decode");
            CHECK(memcmp(&m, &m2, sizeof(m)) == 0, "roundtrip equal");
        }
    }
    CHECK(got == 1, "one frame decoded");

    /* Кадр не влезает в буфер */
    uint8_t tiny[10];
    CHECK(imu_proto_encode(IMU_MSG_ORIENTATION, payload, IMU_ORIENTATION_LEN, tiny,
                           sizeof(tiny)) == 0,
          "encode overflow guard");
}

/* ---------- 3. Протокол: ресинхронизация и битый CRC ---------- */

static void test_proto_resync(void)
{
    uint8_t payload[IMU_ACK_LEN] = {0x80u, 0x00u, 0x00u, 0x00u};
    uint8_t good[16];
    size_t gn = imu_proto_encode(IMU_MSG_ACK, payload, IMU_ACK_LEN, good, sizeof(good));

    uint8_t stream[256];
    size_t pos = 0;
    /* мусор */
    uint8_t garbage[] = {0x00u, 0xFFu, 0xAAu, 0x00u, 0xAAu, 0x55u, 200u};
    memcpy(&stream[pos], garbage, sizeof(garbage));
    pos += sizeof(garbage);
    /* битый кадр (портим CRC) */
    memcpy(&stream[pos], good, gn);
    stream[pos + gn - 1] ^= 0xFFu;
    pos += gn;
    /* хороший кадр */
    memcpy(&stream[pos], good, gn);
    pos += gn;

    imu_decoder_t dec;
    imu_decoder_init(&dec);
    int got = 0;
    for (size_t i = 0; i < pos; i++) {
        uint8_t id, len;
        const uint8_t *pl;
        if (imu_decoder_feed(&dec, stream[i], &id, &pl, &len)) {
            got++;
            CHECK(id == IMU_MSG_ACK && len == IMU_ACK_LEN, "resync frame");
        }
    }
    CHECK(got == 1, "only good frame accepted, got=%d", got);

    /* Доставка по частям */
    imu_decoder_init(&dec);
    got = 0;
    for (size_t i = 0; i < gn / 2; i++) {
        uint8_t id, len;
        const uint8_t *pl;
        if (imu_decoder_feed(&dec, good[i], &id, &pl, &len)) {
            got++;
        }
    }
    CHECK(got == 0, "no premature frame");
    for (size_t i = gn / 2; i < gn; i++) {
        uint8_t id, len;
        const uint8_t *pl;
        if (imu_decoder_feed(&dec, good[i], &id, &pl, &len)) {
            got++;
        }
    }
    CHECK(got == 1, "split frame decoded");
}

/* ---------- 4. Madgwick: кардинальные курсы ---------- */

typedef struct {
    const char *name;
    float mx, my, mz;
    float exp_yaw;
    float exp_enu[3];
} heading_case_t;

static const heading_case_t HEADINGS[] = {
    {"N", 20.0f, 0.0f, -45.0f, 0.0f, {0.0f, 1.0f, 0.0f}},
    {"E", 0.0f, 20.0f, -45.0f, -90.0f, {1.0f, 0.0f, 0.0f}},
    {"S", -20.0f, 0.0f, -45.0f, 180.0f, {0.0f, -1.0f, 0.0f}},
    {"W", 0.0f, -20.0f, -45.0f, 90.0f, {-1.0f, 0.0f, 0.0f}},
};

static void test_madgwick_cardinal(void)
{
    for (size_t h = 0; h < sizeof(HEADINGS) / sizeof(HEADINGS[0]); h++) {
        const heading_case_t *c = &HEADINGS[h];
        madgwick_t f;
        madgwick_init(&f, 0.1f, 50.0f);
        /* Штатный сценарий: алгебраическая инициализация + трекинг 3 с */
        madgwick_init_from_accel_mag(&f, 0, 0, 9.80665f, c->mx, c->my, c->mz);
        for (int i = 0; i < 150; i++) {
            madgwick_update_9(&f, 0, 0, 0, 0, 0, 9.80665f, c->mx, c->my, c->mz);
        }
        float roll, pitch, yaw;
        madgwick_euler_nwu(&f, &roll, &pitch, &yaw);
        CHECK(ang_diff(roll, 0.0f) < 2.0f, "%s roll (%.2f)", c->name, roll);
        CHECK(ang_diff(pitch, 0.0f) < 2.0f, "%s pitch (%.2f)", c->name, pitch);
        CHECK(ang_diff(yaw, c->exp_yaw) < 3.0f, "%s yaw (got %.1f)", c->name, yaw);

        float qw, qx, qy, qz;
        madgwick_quat_ros_enu(&f, &qw, &qx, &qy, &qz);
        float n = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
        CHECK_CLOSE(n, 1.0f, 1e-3f, "%s quat norm", c->name);
        float v[3];
        quat_rot_x(qw, qx, qy, qz, v);
        CHECK(fabsf(v[0] - c->exp_enu[0]) < 0.1f &&
                  fabsf(v[1] - c->exp_enu[1]) < 0.1f && fabsf(v[2] - c->exp_enu[2]) < 0.1f,
              "%s ROS dir (%.2f,%.2f,%.2f)", c->name, v[0], v[1], v[2]);
    }
}

/* ---------- 5. Азимут с компенсацией наклона ---------- */

static void test_tilt_comp(void)
{
    const float exp_az[] = {0.0f, 90.0f, 180.0f, 270.0f};
    for (size_t h = 0; h < 4; h++) {
        const heading_case_t *c = &HEADINGS[h];
        float az = imu_tilt_compensated_azimuth(0, 0, 9.80665f, c->mx, c->my, c->mz, 0.0f);
        CHECK(ang_diff(az, exp_az[h]) < 1.0f, "tilt %s (got %.2f)", c->name, az);
    }
    /* Склонение */
    float az = imu_tilt_compensated_azimuth(0, 0, 9.80665f, 0, 20.0f, -45.0f, 11.5f);
    CHECK(ang_diff(az, 101.5f) < 1.0f, "declination (got %.2f)", az);

    /* Наклон: тангаж вниз 30 град, курс север */
    const float c30 = 0.8660254f, s30 = 0.5f;
    float t_ax = 9.80665f * s30, t_ay = 0.0f, t_az = 9.80665f * c30;
    float t_mx = 20.0f * c30 + (-45.0f) * s30;
    float t_my = 0.0f;
    float t_mz = -20.0f * s30 + (-45.0f) * c30;
    az = imu_tilt_compensated_azimuth(t_ax, t_ay, t_az, t_mx, t_my, t_mz, 0.0f);
    CHECK(ang_diff(az, 0.0f) < 1.5f, "tilted north (got %.2f)", az);
}

/* ---------- 6. Fusion: сходимость ---------- */

static void feed_north(imu_fusion_t *f, int n)
{
    imu_sample_t s = {0, 0, 9.80665f, 0, 0, 0, 20.0f, 0.0f, -45.0f, true, 25.0f};
    imu_result_t r;
    for (int i = 0; i < n; i++) {
        imu_fusion_update(f, &s, &r);
    }
}

static void test_fusion_converge(void)
{
    imu_calib_t cal;
    imu_calib_defaults(&cal);
    imu_fusion_t f;
    imu_fusion_init(&f, &cal);
    feed_north(&f, 500);

    imu_sample_t s = {0, 0, 9.80665f, 0, 0, 0, 20.0f, 0, -45.0f, true, 25.0f};
    imu_result_t r;
    imu_fusion_update(&f, &s, &r);
    CHECK(r.fused_9x, "9-axis mode");
    CHECK(ang_diff(r.yaw_deg, 0.0f) < 4.0f, "fused yaw (%.2f)", r.yaw_deg);
    CHECK(ang_diff(r.azimuth_deg, 11.5f) < 3.0f, "azimuth+decl (%.2f)", r.azimuth_deg);

    /* Потеря мага -> 6 осей, азимут заморожен */
    s.mag_valid = false;
    imu_fusion_update(&f, &s, &r);
    CHECK(!r.fused_9x, "6-axis fallback");
    CHECK(ang_diff(r.azimuth_deg, 11.5f) < 3.0f, "azimuth hold");
}

/* ---------- 7. Калибровка магнитометра ---------- */

static void test_mag_calib(void)
{
    imu_calib_t cal;
    imu_calib_defaults(&cal);
    imu_fusion_t f;
    imu_fusion_init(&f, &cal);

    /* Отмена без применения */
    imu_mag_calib_start(&f, 10);
    imu_mag_calib_feed(&f, 1, 2, 3);
    imu_mag_calib_finish(&f, false);
    CHECK(!f.calib.mag_calibrated, "cancel keeps uncalibrated");

    /* Сбор сферы со смещением hard=(30,-40,50), R=50 */
    imu_mag_calib_start(&f, 120);
    const float hx = 30.0f, hy = -40.0f, hz = 50.0f, R = 50.0f;
    bool done = false;
    for (int i = 0; i < 120; i++) {
        float v[3] = {hx, hy, hz};
        v[i % 3] += ((i / 3) % 2 == 0) ? R : -R;
        done = imu_mag_calib_feed(&f, v[0], v[1], v[2]);
    }
    CHECK(done, "collection completes");
    CHECK(imu_mag_calib_progress(&f) == 100, "progress 100");
    imu_mag_calib_finish(&f, true);
    CHECK(f.calib.mag_calibrated, "calibrated flag");
    CHECK_CLOSE(f.calib.mag_hard[0], hx, 0.5f, "hard x");
    CHECK_CLOSE(f.calib.mag_hard[1], hy, 0.5f, "hard y");
    CHECK_CLOSE(f.calib.mag_hard[2], hz, 0.5f, "hard z");
    CHECK_CLOSE(f.calib.mag_scale[0], 1.0f, 0.02f, "scale x");
    CHECK_CLOSE(f.calib.mag_scale[1], 1.0f, 0.02f, "scale y");
    CHECK_CLOSE(f.calib.mag_scale[2], 1.0f, 0.02f, "scale z");
}

/* ---------- 8. Калибровка гироскопа ---------- */

static void test_gyro_calib(void)
{
    imu_calib_t cal;
    imu_calib_defaults(&cal);
    imu_fusion_t f;
    imu_fusion_init(&f, &cal);

    imu_gyro_calib_start(&f, 50);
    bool done = false;
    for (int i = 0; i < 50; i++) {
        done = imu_gyro_calib_feed(&f, 0.01f, -0.02f, 0.005f);
    }
    CHECK(done, "gyro done");
    CHECK(f.calib.gyro_calibrated, "gyro flag");
    CHECK_CLOSE(f.calib.gyro_bias[0], 0.01f, 1e-6f, "bias x");
    CHECK_CLOSE(f.calib.gyro_bias[1], -0.02f, 1e-6f, "bias y");
    CHECK_CLOSE(f.calib.gyro_bias[2], 0.005f, 1e-6f, "bias z");
}

/* Повернуть fusion от from_deg к to_deg (CCW+, град) со скоростью rate_dps,
 * подавая согласованные гироскоп и магнитометр. В конце - выдержка 2 с. */
static void fusion_rotate_to(imu_fusion_t *f, float from_deg, float to_deg, float rate_dps)
{
    float total = to_deg - from_deg;
    int steps = (int)(fabsf(total) / rate_dps * 50.0f);
    if (steps < 1) {
        steps = 1;
    }
    float wz = ((total >= 0.0f) ? rate_dps : -rate_dps) * T_PI / 180.0f;
    imu_result_t r;
    for (int i = 1; i <= steps; i++) {
        float psi = (from_deg + total * (float)i / (float)steps) * T_PI / 180.0f;
        imu_sample_t s = {0, 0, 9.80665f, 0, 0, wz, 20.0f * cosf(psi),
                          -20.0f * sinf(psi), -45.0f, true, 25.0f};
        imu_fusion_update(f, &s, &r);
    }
    float psi_end = to_deg * T_PI / 180.0f;
    for (int i = 0; i < 100; i++) {
        imu_sample_t s = {0, 0, 9.80665f, 0, 0, 0, 20.0f * cosf(psi_end),
                          -20.0f * sinf(psi_end), -45.0f, true, 25.0f};
        imu_fusion_update(f, &s, &r);
    }
}

/* ---------- 8.5. Трекинг поворота гироскопом ---------- */

static void test_tracking_rotate(void)
{
    imu_calib_t cal;
    imu_calib_defaults(&cal);
    cal.declination_deg = 0.0f;
    imu_fusion_t f;
    imu_fusion_init(&f, &cal);
    feed_north(&f, 100); /* авто-инициализация */

    fusion_rotate_to(&f, 0.0f, -90.0f, 90.0f); /* север -> восток */
    imu_sample_t s = {0, 0, 9.80665f, 0, 0, 0, 0, 20.0f, -45.0f, true, 25.0f};
    imu_result_t r;
    imu_fusion_update(&f, &s, &r);
    CHECK(ang_diff(r.yaw_deg, -90.0f) < 4.0f, "track east yaw (%.1f)", r.yaw_deg);
    CHECK(ang_diff(r.azimuth_deg, 90.0f) < 3.0f, "track east az (%.1f)", r.azimuth_deg);
    float v[3];
    quat_rot_x(r.qw, r.qx, r.qy, r.qz, v);
    CHECK(fabsf(v[0] - 1.0f) < 0.1f && fabsf(v[1]) < 0.1f, "track east dir");

    fusion_rotate_to(&f, -90.0f, -180.0f, 90.0f); /* восток -> юг */
    s.mx = -20.0f;
    s.my = 0.0f;
    imu_fusion_update(&f, &s, &r);
    CHECK(ang_diff(r.yaw_deg, 180.0f) < 4.0f, "track south yaw (%.1f)", r.yaw_deg);

    fusion_rotate_to(&f, -180.0f, -360.0f, 180.0f); /* полный круг обратно */
    s.mx = 20.0f;
    imu_fusion_update(&f, &s, &r);
    CHECK(ang_diff(r.yaw_deg, 0.0f) < 5.0f, "track full circle (%.1f)", r.yaw_deg);
}

/* ---------- 9. ZERO_YAW ---------- */

static void test_zero_yaw(void)
{
    imu_calib_t cal;
    imu_calib_defaults(&cal);
    imu_fusion_t f;
    imu_fusion_init(&f, &cal);
    feed_north(&f, 300);

    /* Довернуть на восток гироскопом (как в реальности) */
    fusion_rotate_to(&f, 0.0f, -90.0f, 90.0f);
    imu_sample_t s = {0, 0, 9.80665f, 0, 0, 0, 0, 20.0f, -45.0f, true, 25.0f};
    imu_result_t r;
    imu_fusion_update(&f, &s, &r);
    CHECK(ang_diff(r.yaw_deg, -90.0f) < 5.0f, "east yaw (%.1f)", r.yaw_deg);
    CHECK(ang_diff(r.azimuth_deg, 101.5f) < 3.0f, "east az (%.1f)", r.azimuth_deg);

    imu_zero_yaw(&f, 0);
    for (int i = 0; i < 50; i++) {
        imu_fusion_update(&f, &s, &r);
    }
    CHECK(ang_diff(r.yaw_deg, 0.0f) < 5.0f, "zeroed yaw (%.1f)", r.yaw_deg);
    CHECK(ang_diff(r.azimuth_deg, 0.0f) < 3.0f, "zeroed az (%.1f)", r.azimuth_deg);

    imu_zero_yaw(&f, 1);
    imu_fusion_update(&f, &s, &r);
    CHECK(ang_diff(r.azimuth_deg, 101.5f) < 3.0f, "cleared az (%.1f)", r.azimuth_deg);
}

/* ---------- 10. Драйверы ---------- */

static int writes_contain(uint16_t dev, uint8_t reg, uint8_t val)
{
    for (int i = 0; i < stub_i2c_write_count(); i++) {
        const stub_i2c_write_t *w = &stub_i2c_writes()[i];
        if (w->dev == dev && w->reg == reg && w->val == val) {
            return 1;
        }
    }
    return 0;
}

static void test_drivers(void)
{
    stub_reset();
    I2C_HandleTypeDef hi2c;

    mpu6050_t mpu;
    CHECK(mpu6050_init(&mpu, &hi2c, 0x68u) == HAL_OK, "mpu init");
    CHECK(writes_contain(0xD0u, 0x1Bu, 0x08u), "mpu gyro cfg 500dps");
    CHECK(writes_contain(0xD0u, 0x1Cu, 0x08u), "mpu accel cfg 4g");
    float ax, ay, az, gx, gy, gz, t;
    CHECK(mpu6050_read(&mpu, &ax, &ay, &az, &gx, &gy, &gz, &t) == HAL_OK, "mpu read");
    CHECK_CLOSE(az, 9.80665f, 0.01f, "mpu az=1g");
    CHECK_CLOSE(gx, 0.0f, 1e-6f, "mpu gx=0");
    CHECK_CLOSE(t, 36.53f, 0.01f, "mpu temp");

    /* MPU6500/клон (GY-521): WHO_AM_I = 0x70, карта регистров та же */
    stub_mpu_set_who(MPU6500_WHO_AM_I_VAL);
    CHECK(mpu6050_init(&mpu, &hi2c, 0x68u) == HAL_OK, "mpu6500 init");
    CHECK(mpu.who_id == MPU6500_WHO_AM_I_VAL, "mpu6500 who id");
    stub_mpu_set_who(MPU6050_WHO_AM_I_VAL);

    /* Чужой ID -> инициализация не проходит */
    stub_mpu_set_who(0x33u);
    CHECK(mpu6050_init(&mpu, &hi2c, 0x68u) != HAL_OK, "mpu bad who");
    stub_mpu_set_who(MPU6050_WHO_AM_I_VAL);

    qmc5883l_t mag;
    CHECK(qmc5883l_init(&mag, &hi2c, 0x0Du) == HAL_OK, "qmc init");
    CHECK(writes_contain(0x1Au, 0x09u, 0x19u), "qmc ctrl1");
    float mx, my, mz;
    CHECK(qmc5883l_read(&mag, &mx, &my, &mz) == HAL_OK, "qmc read");
    CHECK_CLOSE(mx, 20.0f, 0.01f, "qmc x");
    CHECK_CLOSE(my, 0.0f, 0.01f, "qmc y");
    CHECK_CLOSE(mz, -45.0f, 0.01f, "qmc z");

    /* Нет DRDY -> BUSY */
    stub_mag_set_status(0x00u);
    CHECK(qmc5883l_read(&mag, &mx, &my, &mz) == HAL_BUSY, "qmc busy");
    stub_mag_set_status(0x01u);

    /* Нет датчиков */
    stub_i2c_set_present(0, 0);
    CHECK(mpu6050_init(&mpu, &hi2c, 0x68u) != HAL_OK, "mpu absent");
    CHECK(qmc5883l_init(&mag, &hi2c, 0x0Du) != HAL_OK, "qmc absent");
}

/* ---------- 11. Flash ---------- */

static void test_flash(void)
{
    stub_reset();
    imu_calib_t c;
    CHECK(!imu_flash_load(&c), "empty flash");

    imu_calib_defaults(&c);
    c.gyro_bias[0] = 0.0123f;
    c.mag_hard[1] = -5.5f;
    c.declination_deg = 7.25f;
    c.mag_calibrated = 1;
    CHECK(imu_flash_save(&c), "flash save");
    imu_calib_t c2;
    memset(&c2, 0, sizeof(c2));
    CHECK(imu_flash_load(&c2), "flash load");
    CHECK(memcmp(&c, &c2, sizeof(c)) == 0, "flash roundtrip");

    hal_stub_flash_page()[20] ^= 0xFFu;
    CHECK(!imu_flash_load(&c2), "corrupt detected");
}

/* ---------- 12. Интеграция: полный цикл приложения ---------- */

static int tx_find(uint8_t want_id, uint8_t *payload_out, uint8_t *len_out, int occurrence)
{
    imu_decoder_t dec;
    imu_decoder_init(&dec);
    const uint8_t *tx = stub_uart_tx_data();
    size_t n = stub_uart_tx_len();
    int found = 0;
    for (size_t i = 0; i < n; i++) {
        uint8_t id, len;
        const uint8_t *pl;
        if (imu_decoder_feed(&dec, tx[i], &id, &pl, &len)) {
            if (id == want_id) {
                if (found == occurrence) {
                    if (payload_out) {
                        memcpy(payload_out, pl, len);
                    }
                    if (len_out) {
                        *len_out = len;
                    }
                    return 1;
                }
                found++;
            }
        }
    }
    return 0;
}

static int tx_count(uint8_t want_id)
{
    imu_decoder_t dec;
    imu_decoder_init(&dec);
    const uint8_t *tx = stub_uart_tx_data();
    size_t n = stub_uart_tx_len();
    int cnt = 0;
    for (size_t i = 0; i < n; i++) {
        uint8_t id, len;
        const uint8_t *pl;
        if (imu_decoder_feed(&dec, tx[i], &id, &pl, &len) && id == want_id) {
            cnt++;
        }
    }
    return cnt;
}

static void send_cmd(uint8_t id, const uint8_t *p, uint8_t len)
{
    uint8_t frame[IMU_PROTO_MAX_FRAME];
    size_t n = imu_proto_encode(id, p, len, frame, sizeof(frame));
    stub_uart_tx_clear();
    stub_uart_inject_rx(frame, n);
    ImuApp_Process();
}

static void test_app_loop(void)
{
    stub_reset();
    static I2C_HandleTypeDef hi2c;
    static UART_HandleTypeDef huart_data, huart_dbg;
    stub_uart_set_data_handle(&huart_data);

    ImuApp_Init(&hi2c, &huart_data, &huart_dbg);
    ImuApp_CommsStart();

    CHECK(tx_count(IMU_MSG_INFO) >= 1, "boot INFO");

    for (int i = 0; i < 130; i++) {
        ImuApp_Process();
        stub_tick_advance(20);
    }
    CHECK(ImuApp_GetStats()->frames_sent == 130, "130 frames, got %u",
          ImuApp_GetStats()->frames_sent);
    CHECK(tx_count(IMU_MSG_ORIENTATION) == 130, "130 orientation in TX");

    uint8_t pl[IMU_ORIENTATION_LEN];
    /* Последний кадр: ищем последнее вхождение */
    int total = tx_count(IMU_MSG_ORIENTATION);
    CHECK(tx_find(IMU_MSG_ORIENTATION, pl, NULL, total - 1), "last frame found");
    imu_orientation_t m;
    CHECK(imu_orientation_decode(pl, IMU_ORIENTATION_LEN, &m), "last frame decode");
    float qn = sqrtf(m.qw * m.qw + m.qx * m.qx + m.qy * m.qy + m.qz * m.qz);
    CHECK_CLOSE(qn, 1.0f, 1e-3f, "stream quat norm");
    CHECK(ang_diff(m.azimuth_deg, 11.5f) < 4.0f, "stream azimuth (%.1f)", m.azimuth_deg);
    CHECK((m.status & IMU_STATUS_MPU_OK) != 0, "status mpu");
    CHECK((m.status & IMU_STATUS_MAG_OK) != 0, "status mag");
    CHECK((m.status & IMU_STATUS_GYRO_CAL) != 0, "status gyro cal (auto)");
    CHECK((m.status & IMU_STATUS_FUSED_9X) != 0, "status fused");
    CHECK((m.status & IMU_STATUS_MAG_CAL) == 0, "status mag uncal");
    CHECK(m.rate_hz == 50, "stream rate");

    /* PING */
    send_cmd(IMU_CMD_PING, NULL, 0);
    uint8_t ackp[IMU_ACK_LEN];
    CHECK(tx_find(IMU_MSG_ACK, ackp, NULL, 0), "ping ack");
    imu_ack_t ack;
    imu_ack_decode(ackp, IMU_ACK_LEN, &ack);
    CHECK(ack.cmd_id == IMU_CMD_PING && ack.result == IMU_ACK_OK, "ping ok");

    /* SET_RATE */
    uint8_t r100 = 100;
    send_cmd(IMU_CMD_SET_RATE, &r100, 1);
    CHECK(tx_find(IMU_MSG_ACK, ackp, NULL, 0), "rate ack");
    imu_ack_decode(ackp, IMU_ACK_LEN, &ack);
    CHECK(ack.result == IMU_ACK_OK && ack.info == 100, "rate 100");
    CHECK(ImuApp_GetFusion()->calib.rate_hz == 100, "rate applied");
    uint8_t r7 = 7;
    send_cmd(IMU_CMD_SET_RATE, &r7, 1);
    CHECK(tx_find(IMU_MSG_ACK, ackp, NULL, 0), "rate bad ack");
    imu_ack_decode(ackp, IMU_ACK_LEN, &ack);
    CHECK(ack.result == IMU_ACK_ERR_ARG, "rate 7 rejected");
    uint8_t r50 = 50;
    send_cmd(IMU_CMD_SET_RATE, &r50, 1);

    /* GET_INFO */
    send_cmd(IMU_CMD_GET_INFO, NULL, 0);
    CHECK(tx_find(IMU_MSG_INFO, NULL, NULL, 0), "get_info");

    /* Неизвестная команда */
    send_cmd(0xFFu, NULL, 0);
    CHECK(tx_find(IMU_MSG_ACK, ackp, NULL, 0), "unknown ack");
    imu_ack_decode(ackp, IMU_ACK_LEN, &ack);
    CHECK(ack.result == IMU_ACK_ERR_UNKNOWN, "unknown rejected");

    /* ZERO_YAW */
    uint8_t z0 = 0;
    send_cmd(IMU_CMD_ZERO_YAW, &z0, 1);
    stub_tick_advance(20);
    ImuApp_Process();
    total = tx_count(IMU_MSG_ORIENTATION);
    CHECK(total >= 1, "frames after zero");
    CHECK(tx_find(IMU_MSG_ORIENTATION, pl, NULL, total - 1), "zeroed frame");
    imu_orientation_decode(pl, IMU_ORIENTATION_LEN, &m);
    CHECK(ang_diff(m.azimuth_deg, 0.0f) < 4.0f, "zeroed stream az (%.1f)", m.azimuth_deg);
    uint8_t z1 = 1;
    send_cmd(IMU_CMD_ZERO_YAW, &z1, 1);

    /* MAG_CALIB через приложение: качаем оси экстремумами */
    send_cmd(IMU_CMD_MAG_CALIB_START, NULL, 0);
    CHECK(tx_find(IMU_MSG_ACK, ackp, NULL, 0), "mag start ack");
    imu_ack_decode(ackp, IMU_ACK_LEN, &ack);
    CHECK(ack.result == IMU_ACK_OK, "mag start ok");
    for (int i = 0; i < 1500; i++) {
        int16_t v[3] = {0, 0, 0};
        v[i % 3] = (int16_t)(((i / 3) % 2 == 0) ? 600 : -600);
        stub_mag_set_raw(v[0], v[1], v[2]);
        stub_tick_advance(20);
        ImuApp_Process();
        if (i % 300 == 299) {
            stub_uart_tx_clear(); /* не раздуваем буфер */
        }
    }
    stub_mag_set_raw(600, 0, -1350);
    CHECK(!ImuApp_GetFusion()->mag_cal.active, "mag collection done");
    uint8_t save1 = 1;
    send_cmd(IMU_CMD_MAG_CALIB_STOP, &save1, 1);
    CHECK(tx_find(IMU_MSG_ACK, ackp, NULL, 0), "mag stop ack");
    imu_ack_decode(ackp, IMU_ACK_LEN, &ack);
    CHECK(ack.result == IMU_ACK_OK, "mag stop ok");
    CHECK(ImuApp_GetFusion()->calib.mag_calibrated, "app mag calibrated");
    imu_calib_t fc;
    CHECK(imu_flash_load(&fc) && fc.mag_calibrated, "mag calib in flash");

    /* GYRO_CALIB */
    send_cmd(IMU_CMD_GYRO_CALIB, NULL, 0);
    for (int i = 0; i < 120; i++) {
        stub_tick_advance(20);
        ImuApp_Process();
    }
    CHECK(ImuApp_GetFusion()->calib.gyro_calibrated, "app gyro calibrated");

    /* SET_DECLINATION */
    uint8_t dp[4] = {0x00u, 0x00u, 0xA0u, 0x40u}; /* 5.0f LE */
    send_cmd(IMU_CMD_SET_DECLINATION, dp, 4);
    CHECK(tx_find(IMU_MSG_ACK, ackp, NULL, 0), "decl ack");
    imu_ack_decode(ackp, IMU_ACK_LEN, &ack);
    CHECK(ack.result == IMU_ACK_OK, "decl ok");
    CHECK_CLOSE(ImuApp_GetFusion()->calib.declination_deg, 5.0f, 1e-6f, "decl applied");

    /* SAVE_FLASH */
    send_cmd(IMU_CMD_SAVE_FLASH, NULL, 0);
    CHECK(tx_find(IMU_MSG_ACK, ackp, NULL, 0), "save ack");
    imu_ack_decode(ackp, IMU_ACK_LEN, &ack);
    CHECK(ack.result == IMU_ACK_OK, "save ok");
}

/* ---------- 10b. Магнитометр: вариант HMC5883L (автоопределение) ---------- */

static void test_mag_hmc(void)
{
    stub_reset();
    I2C_HandleTypeDef hi2c;
    qmc5883l_t mag;
    float mx = 0, my = 0, mz = 0;

    /* HMC на 0x1E, QMC-адреса пусто: автоопределение должна нагнуться */
    stub_mag_set_hmc(1);
    stub_mag_set_ut(20.0f, 0.0f, -45.0f);
    CHECK(qmc5883l_init(&mag, &hi2c, 0x0Du) == HAL_OK, "hmc init autodetect");
    CHECK(mag.type == MAG5883_HMC, "hmc type detected");
    CHECK(mag.dev_addr == 0x3Cu, "hmc i2c addr");
    CHECK(writes_contain(0x3Cu, 0x00u, 0x7Cu), "hmc CRA (75 Гц, 8x ср.)");
    CHECK(writes_contain(0x3Cu, 0x01u, 0xE0u), "hmc CRB (±8.1 G)");
    CHECK(writes_contain(0x3Cu, 0x02u, 0x00u), "hmc continuous");
    CHECK(qmc5883l_read(&mag, &mx, &my, &mz) == HAL_OK, "hmc read");
    /* ±8.1 G: 1 LSB ~ 0.43 мкТл, допуск на квантование */
    CHECK_CLOSE(mx, 20.0f, 0.5f, "hmc x");
    CHECK_CLOSE(my, 0.0f, 0.5f, "hmc y");
    CHECK_CLOSE(mz, -45.0f, 0.5f, "hmc z");

    /* HMC на шине нет */
    stub_reset();
    stub_mag_set_hmc(1);
    stub_i2c_set_present(1, 0);
    CHECK(qmc5883l_init(&mag, &hi2c, 0x0Du) != HAL_OK, "hmc absent");

    /* QMC и HMC одновременно: приоритет у QMC на 0x0D */
    stub_reset();
    stub_mag_set_dual(1);
    stub_mag_set_ut(20.0f, 0.0f, -45.0f);
    CHECK(qmc5883l_init(&mag, &hi2c, 0x0Du) == HAL_OK, "dual init");
    CHECK(mag.type == MAG5883_QMC, "dual prefers QMC");
    CHECK(mag.dev_addr == 0x1Au, "dual qmc addr");
    CHECK(qmc5883l_read(&mag, &mx, &my, &mz) == HAL_OK, "dual read");
    CHECK_CLOSE(mx, 20.0f, 0.01f, "dual x");
    CHECK_CLOSE(mz, -45.0f, 0.01f, "dual z");
}

/* ---------- 12. Диагностика (вывод в отладочный UART) ---------- */

static void test_diag(void)
{
    stub_reset();
    static I2C_HandleTypeDef hi2c;
    static UART_HandleTypeDef huart_data, huart_dbg;
    stub_uart_set_data_handle(&huart_data);
    stub_uart_set_dbg_handle(&huart_dbg);

    ImuApp_Init(&hi2c, &huart_data, &huart_dbg);
    ImuApp_CommsStart();

    const char *d = (const char *)stub_dbg_peek();
    CHECK(strstr(d, "IMU DIAG: BOOT") != NULL, "diag banner");
    CHECK(strstr(d, "I2C1 SCAN: 0x68=1 0x69=0 0x0D=1 0x1E=0") != NULL,
          "i2c scan line");
    CHECK(strstr(d, "WHO_AM_I: 0x68") != NULL, "mpu who am i");
    CHECK(strstr(d, "CHIP_ID: 0xff") != NULL, "mag chip id");
    CHECK(strstr(d, "MPU data: a=") != NULL, "mpu first read");
    CHECK(strstr(d, "MAG data: (") != NULL, "mag first read");
    /* Цифры должны печататься по-настоящему (регрессия: %.2f литералом) */
    CHECK(strstr(d, "MPU data: a=(0.00 0.00 9.81)") != NULL, "mpu data values");
    CHECK(strstr(d, "MAG data: (20.0 0.0 -45.0) uT") != NULL, "mag data values");
    CHECK(strstr(d, "%.") == NULL, "no literal format specs");

    /* Периодический статус после 10 с */
    stub_tick_set(10000);
    ImuApp_Process();
    d = (const char *)stub_dbg_peek();
    CHECK(strstr(d, "ST t=10 s") != NULL, "periodic status line");
    CHECK(strstr(d, "frames=") != NULL, "frames in status");
    CHECK(strstr(d, "send_age=") != NULL, "send_age in status");

    /* Датчиков нет: диагностика показывает пустой scan и "не найден" */
    stub_reset();
    stub_i2c_set_present(0, 0);
    stub_uart_set_dbg_handle(&huart_dbg);
    ImuApp_Init(&hi2c, &huart_data, &huart_dbg);
    d = (const char *)stub_dbg_peek();
    CHECK(strstr(d, "I2C1 SCAN: 0x68=0 0x69=0 0x0D=0 0x1E=0") != NULL,
          "scan all absent");
    CHECK(strstr(d, "MPU data: датчик не найден") != NULL, "mpu not detected");
    CHECK(strstr(d, "MAG data: датчик не найден") != NULL, "mag not detected");
}

int main(void)
{
    test_crc();
    test_proto_roundtrip();
    test_proto_resync();
    test_madgwick_cardinal();
    test_tilt_comp();
    test_fusion_converge();
    test_mag_calib();
    test_gyro_calib();
    test_tracking_rotate();
    test_zero_yaw();
    test_drivers();
    test_mag_hmc();
    test_diag();
    test_flash();
    test_app_loop();

    printf("checks: %d, failures: %d\n", s_checks, s_failures);
    return (s_failures == 0) ? 0 : 1;
}
