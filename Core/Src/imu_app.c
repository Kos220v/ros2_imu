#include "imu_app.h"

#include <string.h>

#include "debug_log.h"
#include "imu_config.h"
#include "imu_flash.h"
#include "imu_protocol.h"
#include "mpu6050.h"
#include "qmc5883l.h"

#ifndef IMU_HOST_TEST
#include "stm32f3xx_hal.h" /* __disable_irq / __enable_irq */
#endif

typedef struct {
    uint8_t id;
    uint8_t len;
    uint8_t payload[IMU_PROTO_MAX_PAYLOAD];
    volatile uint8_t pending;
} imu_rx_cmd_t;

static I2C_HandleTypeDef *s_hi2c;
static UART_HandleTypeDef *s_huart;
static mpu6050_t s_mpu;
static qmc5883l_t s_mag;
static imu_fusion_t s_fusion;
static imu_decoder_t s_decoder;
static imu_rx_cmd_t s_cmd;
static uint8_t s_rx_byte;
static imu_app_stats_t s_stats;
static uint32_t s_next_tick;
static uint32_t s_last_mag_ok_tick;
static float s_last_mx, s_last_my, s_last_mz;
static float s_last_temp;
static uint8_t s_mpu_ok, s_mag_ok;
static uint8_t s_mpu_err_cnt, s_mag_err_cnt;
static uint8_t s_last_progress_sent;

/* Диагностика */
#define IMU_DIAG_PERIOD_MS 10000u
static uint32_t s_last_send_tick;
static uint32_t s_last_mpu_ok_tick;
static uint32_t s_last_diag_tick;
static imu_result_t s_last_res;

/* ---------- Передача ---------- */

static void imu_send(uint8_t msg_id, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[IMU_PROTO_MAX_FRAME];
    size_t n = imu_proto_encode(msg_id, payload, len, frame, sizeof(frame));
    if (n > 0) {
        if (HAL_UART_Transmit(s_huart, frame, (uint16_t)n, IMU_UART_TX_TIMEOUT_MS) == HAL_OK) {
            s_last_send_tick = HAL_GetTick();
            if (msg_id == IMU_MSG_ORIENTATION) {
                s_stats.frames_sent++;
            }
        }
    }
}

static void imu_send_ack(uint8_t cmd_id, uint8_t result, uint16_t info)
{
    imu_ack_t ack = {cmd_id, result, info};
    uint8_t p[IMU_ACK_LEN];
    imu_ack_encode(&ack, p);
    imu_send(IMU_MSG_ACK, p, IMU_ACK_LEN);
}

static void imu_send_info(void)
{
    imu_info_t info;
    memset(&info, 0, sizeof(info));
    info.fw_major = IMU_FW_VERSION_MAJOR;
    info.fw_minor = IMU_FW_VERSION_MINOR;
    info.fw_patch = IMU_FW_VERSION_PATCH;
    info.uptime_ms = HAL_GetTick();
    strncpy(info.board, IMU_BOARD_NAME, sizeof(info.board) - 1);
    info.mpu_ok = s_mpu_ok;
    info.mag_ok = s_mag_ok;
    info.mag_cal = s_fusion.calib.mag_calibrated;
    info.gyro_cal = s_fusion.calib.gyro_calibrated;
    info.declination_deg = s_fusion.calib.declination_deg;
    info.rate_hz = s_fusion.calib.rate_hz;
    uint8_t p[IMU_INFO_LEN];
    imu_info_encode(&info, p);
    imu_send(IMU_MSG_INFO, p, IMU_INFO_LEN);
}

static uint8_t imu_calib_state(void)
{
    if (s_fusion.mag_cal.active) {
        return IMU_CALIB_MAG_RUN;
    }
    if (s_fusion.gyro_cal.active) {
        return IMU_CALIB_GYRO_RUN;
    }
    return IMU_CALIB_IDLE;
}

static void imu_send_calib(void)
{
    imu_calib_msg_t m;
    m.state = imu_calib_state();
    m.progress_pct = s_fusion.mag_cal.active ? imu_mag_calib_progress(&s_fusion)
                       : s_fusion.gyro_cal.active
                           ? (uint8_t)((s_fusion.gyro_cal.count * 100u) /
                                       (s_fusion.gyro_cal.target ? s_fusion.gyro_cal.target : 1u))
                           : 100u;
    m.reserved[0] = 0;
    m.reserved[1] = 0;
    m.mag_hard_x = s_fusion.calib.mag_hard[0];
    m.mag_hard_y = s_fusion.calib.mag_hard[1];
    m.mag_hard_z = s_fusion.calib.mag_hard[2];
    m.mag_scale_x = s_fusion.calib.mag_scale[0];
    m.mag_scale_y = s_fusion.calib.mag_scale[1];
    m.mag_scale_z = s_fusion.calib.mag_scale[2];
    m.gyro_bias_x = s_fusion.calib.gyro_bias[0];
    m.gyro_bias_y = s_fusion.calib.gyro_bias[1];
    m.gyro_bias_z = s_fusion.calib.gyro_bias[2];
    uint8_t p[IMU_CALIB_LEN];
    imu_calib_msg_encode(&m, p);
    imu_send(IMU_MSG_CALIB, p, IMU_CALIB_LEN);
}

/* ---------- Приём команд (прерывание) ---------- */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_huart) {
        return;
    }
    /* Сразу перевооружаем приём */
    (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);

    uint8_t id;
    const uint8_t *payload;
    uint8_t len;
    if (imu_decoder_feed(&s_decoder, s_rx_byte, &id, &payload, &len)) {
        if (!s_cmd.pending && len <= IMU_PROTO_MAX_PAYLOAD) {
            s_cmd.id = id;
            s_cmd.len = len;
            if (len > 0) {
                memcpy(s_cmd.payload, payload, len);
            }
            s_cmd.pending = 1;
        }
    }
}

/* ---------- Обработка команд (основной цикл) ---------- */

static uint8_t rate_is_valid(uint8_t r)
{
    return (r == 10 || r == 25 || r == 50 || r == 100);
}

static void imu_handle_cmd(uint8_t id, const uint8_t *p, uint8_t len)
{
    s_stats.cmds_rx++;
    switch (id) {
    case IMU_CMD_PING:
        imu_send_ack(id, IMU_ACK_OK, 0);
        break;
    case IMU_CMD_SET_RATE:
        if (len != 1 || !rate_is_valid(p[0])) {
            imu_send_ack(id, IMU_ACK_ERR_ARG, 0);
        } else {
            imu_fusion_set_rate(&s_fusion, p[0]);
            imu_send_ack(id, IMU_ACK_OK, p[0]);
        }
        break;
    case IMU_CMD_MAG_CALIB_START:
        if (s_fusion.gyro_cal.active) {
            imu_send_ack(id, IMU_ACK_ERR_STATE, 0);
        } else {
            imu_mag_calib_start(&s_fusion, IMU_MAG_CALIB_SAMPLES);
            s_last_progress_sent = 0;
            imu_send_ack(id, IMU_ACK_OK, 0);
        }
        break;
    case IMU_CMD_MAG_CALIB_STOP:
        if (len != 1 || (p[0] != 0 && p[0] != 1)) {
            imu_send_ack(id, IMU_ACK_ERR_ARG, 0);
        } else if (s_fusion.mag_cal.count == 0 && !s_fusion.mag_cal.active) {
            imu_send_ack(id, IMU_ACK_ERR_STATE, 0);
        } else {
            bool apply = (p[0] == 1);
            imu_mag_calib_finish(&s_fusion, apply);
            uint8_t res = IMU_ACK_OK;
            if (apply) {
                if (!imu_flash_save(&s_fusion.calib)) {
                    res = IMU_ACK_ERR_HW;
                }
            }
            imu_send_ack(id, res, 0);
            imu_send_calib();
        }
        break;
    case IMU_CMD_GYRO_CALIB:
        if (s_fusion.mag_cal.active) {
            imu_send_ack(id, IMU_ACK_ERR_STATE, 0);
        } else {
            imu_gyro_calib_start(&s_fusion, IMU_GYRO_CALIB_SAMPLES);
            imu_send_ack(id, IMU_ACK_OK, 0);
        }
        break;
    case IMU_CMD_ZERO_YAW: {
        uint8_t mode = 0;
        if (len == 1) {
            mode = p[0];
        } else if (len != 0) {
            imu_send_ack(id, IMU_ACK_ERR_ARG, 0);
            break;
        }
        if (mode > 1) {
            imu_send_ack(id, IMU_ACK_ERR_ARG, 0);
        } else {
            imu_zero_yaw(&s_fusion, mode);
            imu_send_ack(id, IMU_ACK_OK, mode);
        }
        break;
    }
    case IMU_CMD_SET_DECLINATION:
        if (len != 4) {
            imu_send_ack(id, IMU_ACK_ERR_ARG, 0);
        } else {
            float d = imu_get_f32le(p);
            if (d < -30.0f || d > 30.0f) {
                imu_send_ack(id, IMU_ACK_ERR_ARG, 0);
            } else {
                s_fusion.calib.declination_deg = d;
                imu_send_ack(id, IMU_ACK_OK, 0);
            }
        }
        break;
    case IMU_CMD_SAVE_FLASH:
        imu_send_ack(id, imu_flash_save(&s_fusion.calib) ? IMU_ACK_OK : IMU_ACK_ERR_HW, 0);
        break;
    case IMU_CMD_GET_INFO:
        imu_send_info();
        break;
    case IMU_CMD_GET_CALIB:
        imu_send_calib();
        break;
    default:
        imu_send_ack(id, IMU_ACK_ERR_UNKNOWN, 0);
        break;
    }
}

/* ---------- Диагностика (вывод в USART1) ---------- */

static int probe_addr(uint8_t addr7)
{
    return HAL_I2C_IsDeviceReady(s_hi2c, (uint16_t)addr7 << 1, 3,
                                 IMU_I2C_TIMEOUT_MS) == HAL_OK;
}

static void imu_diag_boot(void)
{
    dbg_print("\r\n=== IMU DIAG: BOOT ===\r\n");
    dbg_printf("CLK: SYSCLK %u MHz (HCLK /2, PCLK1 /1), I2C1 <- HSI 8 MHz\r\n",
               (unsigned)(SystemCoreClock / 1000000u));
    dbg_printf("I2C1 SCAN: 0x68=%d 0x69=%d 0x0D=%d 0x1E=%d\r\n",
               probe_addr(0x68u), probe_addr(0x69u),
               probe_addr(0x0Du), probe_addr(0x1Eu));
    uint8_t who = 0;
    if (probe_addr(IMU_MPU6050_ADDR7)) {
        (void)HAL_I2C_Mem_Read(s_hi2c, (uint16_t)IMU_MPU6050_ADDR7 << 1, 0x75u,
                               I2C_MEMADD_SIZE_8BIT, &who, 1, IMU_I2C_TIMEOUT_MS);
        dbg_printf("MPU WHO_AM_I: 0x%02x (ожидаем 0x68 при AD0=GND)\r\n",
                   (unsigned)who);
    }
    uint8_t cid = 0;
    if (probe_addr(0x0Du)) {
        (void)HAL_I2C_Mem_Read(s_hi2c, 0x1Au, 0x0Du, I2C_MEMADD_SIZE_8BIT,
                               &cid, 1, IMU_I2C_TIMEOUT_MS);
        dbg_printf("MAG@0x0D CHIP_ID: 0x%02x (ожидаем 0xff = QMC5883L/HA5883)\r\n",
                   (unsigned)cid);
    }
    uint8_t id3[3] = {0, 0, 0};
    if (probe_addr(0x1Eu)) {
        (void)HAL_I2C_Mem_Read(s_hi2c, 0x3Cu, 0x0Au, I2C_MEMADD_SIZE_8BIT, &id3[0],
                               1, IMU_I2C_TIMEOUT_MS);
        (void)HAL_I2C_Mem_Read(s_hi2c, 0x3Cu, 0x0Bu, I2C_MEMADD_SIZE_8BIT, &id3[1],
                               1, IMU_I2C_TIMEOUT_MS);
        (void)HAL_I2C_Mem_Read(s_hi2c, 0x3Cu, 0x0Cu, I2C_MEMADD_SIZE_8BIT, &id3[2],
                               1, IMU_I2C_TIMEOUT_MS);
        dbg_printf("MAG@0x1E ID: %c%c%c (ожидаем 'H43' = HMC5883L)\r\n",
                   (char)id3[0], (char)id3[1], (char)id3[2]);
    }
}

static void imu_diag_sensors(void)
{
    if (s_mpu_ok) {
        float ax, ay, az, gx, gy, gz, t;
        if (mpu6050_read(&s_mpu, &ax, &ay, &az, &gx, &gy, &gz, &t) == HAL_OK) {
            dbg_printf("MPU data: a=(%.2f %.2f %.2f) m/s2 g=(%.3f %.3f %.3f) "
                       "rad/s T=%.1f C\r\n",
                       ax, ay, az, gx, gy, gz, t);
        } else {
            dbg_print("MPU data: ошибка чтения (таймаут I2C?)\r\n");
        }
    } else {
        dbg_print("MPU data: датчик не найден\r\n");
    }
    if (s_mag_ok) {
        float mx, my, mz;
        HAL_StatusTypeDef st = qmc5883l_read(&s_mag, &mx, &my, &mz);
        if (st == HAL_OK) {
            dbg_printf("MAG data: (%.1f %.1f %.1f) uT | %s @0x%02x\r\n",
                       mx, my, mz,
                       s_mag.type == MAG5883_HMC ? "HMC5883L" : "QMC5883L/HA5883",
                       (unsigned)(s_mag.dev_addr >> 1));
        } else {
            dbg_print("MAG data: нет свежих данных (HAL_BUSY) или ошибка\r\n");
        }
    } else {
        dbg_print("MAG data: датчик не найден\r\n");
    }
    dbg_print("=== IMU DIAG: BOOT END ===\r\n");
}

static void imu_diag_status(uint32_t now)
{
    const imu_result_t *r = &s_last_res;
    dbg_printf("ST t=%u s | MPU %s (err %u) MAG %s (err %u)\r\n",
               (unsigned)(now / 1000u),
               s_mpu_ok ? "ok" : "FAIL", (unsigned)s_stats.mpu_errors,
               s_mag_ok ? (s_mag.type == MAG5883_HMC ? "ok(HMC)" : "ok(QMC)")
                        : "FAIL",
               (unsigned)s_stats.mag_errors);
    dbg_printf("  q=(%.3f %.3f %.3f %.3f) rpy=(%.1f %.1f %.1f) az=%.1f deg\r\n",
               r->qw, r->qx, r->qy, r->qz, r->roll_deg, r->pitch_deg,
               r->yaw_deg, r->azimuth_deg);
    dbg_printf("  g=(%.3f %.3f %.3f) rad/s a=(%.2f %.2f %.2f) m/s2 T=%.1f C\r\n",
               r->wx, r->wy, r->wz, r->ax, r->ay, r->az, s_last_temp);
    uint32_t mpu_age = s_mpu_ok ? (uint32_t)(now - s_last_mpu_ok_tick) : 9999u;
    uint32_t mag_age = s_mag_ok ? (uint32_t)(now - s_last_mag_ok_tick) : 9999u;
    uint32_t send_age = s_stats.frames_sent > 0
                            ? (uint32_t)(now - s_last_send_tick)
                            : 9999u;
    dbg_printf("  m=(%.1f %.1f %.1f) uT %s | %u Hz frames=%u mpu_age=%u ms "
               "mag_age=%u ms send_age=%u ms\r\n",
               s_last_mx, s_last_my, s_last_mz,
               r->fused_9x ? "9x" : "6x", (unsigned)s_fusion.calib.rate_hz,
               (unsigned)s_stats.frames_sent, (unsigned)mpu_age, (unsigned)mag_age,
               (unsigned)send_age);
}

/* ---------- Инициализация ---------- */

void ImuApp_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart_data,
                 UART_HandleTypeDef *huart_dbg)
{
    s_hi2c = hi2c;
    s_huart = huart_data;
    memset(&s_stats, 0, sizeof(s_stats));
    memset(&s_cmd, 0, sizeof(s_cmd));
    imu_decoder_init(&s_decoder);

    dbg_init(huart_dbg);
    dbg_printf("\r\nIMU %s fw %d.%d.%d\r\n", IMU_BOARD_NAME, IMU_FW_VERSION_MAJOR,
               IMU_FW_VERSION_MINOR, IMU_FW_VERSION_PATCH);
    imu_diag_boot();

    /* Калибровка из flash или defaults */
    imu_calib_t calib;
    if (imu_flash_load(&calib)) {
        dbg_print("CALIB: loaded from flash\r\n");
    } else {
        imu_calib_defaults(&calib);
        dbg_print("CALIB: defaults (no flash record)\r\n");
    }
    imu_fusion_init(&s_fusion, &calib);

    /* Датчики (по 3 попытки) */
    s_mpu_ok = 0;
    for (int i = 0; i < 3 && !s_mpu_ok; i++) {
        if (mpu6050_init(&s_mpu, s_hi2c, IMU_MPU6050_ADDR7) == HAL_OK) {
            s_mpu_ok = 1;
        } else {
            HAL_Delay(50);
        }
    }
    dbg_printf("MPU6050: %s\r\n", s_mpu_ok ? "OK" : "FAIL");

    s_mag_ok = 0;
    for (int i = 0; i < 3 && !s_mag_ok; i++) {
        if (qmc5883l_init(&s_mag, s_hi2c, IMU_QMC5883L_ADDR7) == HAL_OK) {
            s_mag_ok = 1;
        } else {
            HAL_Delay(50);
        }
    }
    if (s_mag_ok) {
        dbg_printf("MAG: %s @0x%02X OK\r\n",
                   s_mag.type == MAG5883_HMC ? "HMC5883L" : "QMC5883L/HA5883",
                   (unsigned)(s_mag.dev_addr >> 1));
    } else {
        dbg_print("MAG: FAIL\r\n");
    }
    imu_diag_sensors();

    /* Если гироскоп не калиброван - автокалибровка при старте (не трогать плату!) */
    if (s_mpu_ok && !s_fusion.calib.gyro_calibrated) {
        dbg_print("GYRO: auto-calibration, keep still...\r\n");
        imu_gyro_calib_start(&s_fusion, IMU_GYRO_CALIB_SAMPLES);
    }

    s_next_tick = HAL_GetTick();
    s_last_mag_ok_tick = 0;

    /* Приветственный INFO роботу */
    if (s_huart) {
        imu_send_info();
    }
}

void ImuApp_CommsStart(void)
{
    if (!s_huart) {
        return;
    }
#ifndef IMU_HOST_TEST
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
#endif
    (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

/* ---------- Основной цикл ---------- */

static void imu_poll_sensors(imu_sample_t *s, uint32_t now)
{
    /* Последние валидные данные MPU (переживут единичные сбои шины). */
    static float last_ax = 0.0f, last_ay = 0.0f, last_az = 9.80665f;
    static float last_gx = 0.0f, last_gy = 0.0f, last_gz = 0.0f;
    float ax = last_ax, ay = last_ay, az = last_az;
    float gx = last_gx, gy = last_gy, gz = last_gz, t = 25.0f;
    if (s_mpu_ok && mpu6050_read(&s_mpu, &ax, &ay, &az, &gx, &gy, &gz, &t) == HAL_OK) {
        s_mpu_ok = 1;
        s_mpu_err_cnt = 0;
        s_last_temp = t;
        s_last_mpu_ok_tick = now;
        last_ax = ax;
        last_ay = ay;
        last_az = az;
        last_gx = gx;
        last_gy = gy;
        last_gz = gz;
    } else {
        s_stats.mpu_errors++;
        if (++s_mpu_err_cnt >= 5) {
            /* Попытка реинициализации */
            if (mpu6050_init(&s_mpu, s_hi2c, IMU_MPU6050_ADDR7) == HAL_OK) {
                s_mpu_ok = 1;
                s_mpu_err_cnt = 0;
            }
        }
        if (!s_mpu_ok) {
            return; /* без MPU делать нечего */
        }
    }

    s->ax = ax;
    s->ay = ay;
    s->az = az;
    s->gx = gx;
    s->gy = gy;
    s->gz = gz;
    s->temp_c = s_last_temp;

    /* Магнитометр: HAL_BUSY = свежих нет, берём прошлые */
    float mx, my, mz;
    HAL_StatusTypeDef mst = HAL_ERROR;
    if (s_mag_ok || s_mag.ok) {
        mst = qmc5883l_read(&s_mag, &mx, &my, &mz);
    }
    if (mst == HAL_OK) {
        s_last_mx = mx;
        s_last_my = my;
        s_last_mz = mz;
        s_last_mag_ok_tick = now;
        s_mag_ok = 1;
        s_mag_err_cnt = 0;
        s->mag_valid = true;
    } else if (mst == HAL_BUSY && s_mag_ok) {
        s->mag_valid = true; /* повтор прошлого значения */
        mx = s_last_mx;
        my = s_last_my;
        mz = s_last_mz;
    } else {
        s_stats.mag_errors++;
        if (++s_mag_err_cnt >= 10) {
            if (qmc5883l_init(&s_mag, s_hi2c, IMU_QMC5883L_ADDR7) == HAL_OK) {
                s_mag_ok = 1;
                s_mag_err_cnt = 0;
            } else {
                s_mag_ok = 0;
            }
        }
        /* Протухшие данные старше 500 мс считаем невалидными */
        s->mag_valid = (s_mag_ok && (now - s_last_mag_ok_tick) < 500u);
    }
    s->mx = s_last_mx;
    s->my = s_last_my;
    s->mz = s_last_mz;
}

static void imu_stream(const imu_result_t *r, uint32_t now)
{
    imu_orientation_t m;
    m.ts_ms = now;
    m.qw = r->qw;
    m.qx = r->qx;
    m.qy = r->qy;
    m.qz = r->qz;
    m.roll_deg = r->roll_deg;
    m.pitch_deg = r->pitch_deg;
    m.yaw_deg = r->yaw_deg;
    m.azimuth_deg = r->azimuth_deg;
    m.wx = r->wx;
    m.wy = r->wy;
    m.wz = r->wz;
    m.ax = r->ax;
    m.ay = r->ay;
    m.az = r->az;
    m.mx = r->mx;
    m.my = r->my;
    m.mz = r->mz;
    m.temp_c = s_last_temp;
    m.status = (uint8_t)((s_mpu_ok ? IMU_STATUS_MPU_OK : 0) |
                         (s_mag_ok ? IMU_STATUS_MAG_OK : 0) |
                         (s_fusion.calib.mag_calibrated ? IMU_STATUS_MAG_CAL : 0) |
                         (s_fusion.calib.gyro_calibrated ? IMU_STATUS_GYRO_CAL : 0) |
                         (r->fused_9x ? IMU_STATUS_FUSED_9X : 0));
    m.calib_state = imu_calib_state();
    m.rate_hz = s_fusion.calib.rate_hz;
    m.reserved = 0;
    uint8_t p[IMU_ORIENTATION_LEN];
    imu_orientation_encode(&m, p);
    imu_send(IMU_MSG_ORIENTATION, p, IMU_ORIENTATION_LEN);
}

void ImuApp_Process(void)
{
    /* 1. Команды из ISR */
    if (s_cmd.pending) {
        uint8_t id = s_cmd.id, len = s_cmd.len;
        uint8_t payload[IMU_PROTO_MAX_PAYLOAD];
        __disable_irq();
        memcpy(payload, s_cmd.payload, len);
        s_cmd.pending = 0;
        __enable_irq();
        imu_handle_cmd(id, payload, len);
    }

    /* 1b. Периодическая диагностика (каждые IMU_DIAG_PERIOD_MS) */
    {
        uint32_t now_d = HAL_GetTick();
        if ((uint32_t)(now_d - s_last_diag_tick) >= IMU_DIAG_PERIOD_MS) {
            s_last_diag_tick = now_d;
            imu_diag_status(now_d);
        }
    }

    /* 2. Планировщик опроса по тикам */
    uint32_t now = HAL_GetTick();
    uint32_t period = 1000u / (uint32_t)s_fusion.calib.rate_hz;
    if ((int32_t)(now - s_next_tick) < 0) {
        return;
    }
    s_next_tick += period;
    if ((int32_t)(now - s_next_tick) > (int32_t)period) {
        s_next_tick = now; /* отстали больше чем на период - догоняем */
    }

    /* 3. Опрос датчиков */
    imu_sample_t sample;
    memset(&sample, 0, sizeof(sample));
    imu_poll_sensors(&sample, now);
    if (!s_mpu_ok && s_stats.frames_sent == 0) {
        /* MPU так и не поднялся - не спамим, ждём */
        return;
    }

    /* 4. Сбор калибровок (сырые векторы в осях корпуса) */
    if (s_fusion.gyro_cal.active) {
        float gx = sample.gx, gy = sample.gy, gz = sample.gz;
        imu_apply_mount(&gx, &gy, &gz);
        if (imu_gyro_calib_feed(&s_fusion, gx, gy, gz)) {
            dbg_print("GYRO: calibrated\r\n");
            imu_send_calib();
        }
    }
    if (s_fusion.mag_cal.active && sample.mag_valid) {
        float mx = sample.mx, my = sample.my, mz = sample.mz;
        imu_apply_mount(&mx, &my, &mz);
        if (imu_mag_calib_feed(&s_fusion, mx, my, mz)) {
            dbg_print("MAG: collection done, send STOP to apply\r\n");
            imu_send_calib();
        } else {
            uint8_t pr = imu_mag_calib_progress(&s_fusion);
            if ((uint8_t)(pr - s_last_progress_sent) >= 5u) {
                s_last_progress_sent = pr;
                imu_send_calib();
            }
        }
    }

    /* 5. Fusion + выдача */
    imu_result_t res;
    imu_fusion_update(&s_fusion, &sample, &res);
    s_last_res = res;
    imu_stream(&res, now);
}

const imu_fusion_t *ImuApp_GetFusion(void)
{
    return &s_fusion;
}

const imu_app_stats_t *ImuApp_GetStats(void)
{
    return &s_stats;
}
