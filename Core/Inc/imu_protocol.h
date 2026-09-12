/* Бинарный протокол STM32 <-> робот по UART.
 *
 * Кадр (порядок байт little-endian):
 *   [0]     0xAA
 *   [1]     0x55
 *   [2]     LEN = 1 + длина payload
 *   [3]     MSG_ID
 *   [4..]   PAYLOAD (0..96 байт)
 *   [..]    CRC16-CCITT (poly 0x1021, init 0xFFFF) от [LEN, ID, PAYLOAD], мл. байт первым
 *
 * Сообщения STM32 -> робот:
 *   0x01 ORIENTATION (80 байт) - кватернион, углы, азимут, сырые данные
 *   0x02 INFO (36 байт)        - версия прошивки, статусы
 *   0x03 ACK (4 байта)         - ответ на команду
 *   0x04 CALIB (40 байт)       - калибровки и прогресс калибровки
 *
 * Команды робот -> STM32:
 *   0x80 PING            (0 байт) -> ACK
 *   0x81 SET_RATE        (1 байт: 10/25/50/100) -> ACK
 *   0x82 MAG_CALIB_START (0 байт) -> ACK, далее поток CALIB с прогрессом
 *   0x83 MAG_CALIB_STOP  (1 байт: 0=отменить, 1=применить и сохранить) -> ACK+CALIB
 *   0x84 GYRO_CALIB      (0 байт; плату не трогать ~2 c) -> ACK+CALIB
 *   0x85 ZERO_YAW        (1 байт: 0=обнулить курс, 1=снять смещение) -> ACK
 *   0x86 SET_DECLINATION (4 байта float, град) -> ACK
 *   0x87 SAVE_FLASH      (0 байт) -> ACK
 *   0x88 GET_INFO        (0 байт) -> INFO
 *   0x89 GET_CALIB       (0 байт) -> CALIB
 *
 * Модуль платформо-независимый (без HAL): собирается и для STM32, и для хост-тестов.
 */
#ifndef IMU_PROTOCOL_H
#define IMU_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_PROTO_SYNC0 0xAAu
#define IMU_PROTO_SYNC1 0x55u
#define IMU_PROTO_MAX_PAYLOAD 96u
#define IMU_PROTO_MAX_FRAME (IMU_PROTO_MAX_PAYLOAD + 6u)

/* --- Сообщения STM32 -> робот --- */
#define IMU_MSG_ORIENTATION 0x01u
#define IMU_MSG_INFO 0x02u
#define IMU_MSG_ACK 0x03u
#define IMU_MSG_CALIB 0x04u

#define IMU_ORIENTATION_LEN 80u
#define IMU_INFO_LEN 36u
#define IMU_ACK_LEN 4u
#define IMU_CALIB_LEN 40u

/* --- Команды робот -> STM32 --- */
#define IMU_CMD_PING 0x80u
#define IMU_CMD_SET_RATE 0x81u
#define IMU_CMD_MAG_CALIB_START 0x82u
#define IMU_CMD_MAG_CALIB_STOP 0x83u
#define IMU_CMD_GYRO_CALIB 0x84u
#define IMU_CMD_ZERO_YAW 0x85u
#define IMU_CMD_SET_DECLINATION 0x86u
#define IMU_CMD_SAVE_FLASH 0x87u
#define IMU_CMD_GET_INFO 0x88u
#define IMU_CMD_GET_CALIB 0x89u

/* --- Коды результата в ACK --- */
#define IMU_ACK_OK 0u
#define IMU_ACK_ERR_ARG 1u     /* неверный аргумент */
#define IMU_ACK_ERR_STATE 2u   /* недопустимо в текущем состоянии */
#define IMU_ACK_ERR_HW 3u      /* ошибка железа (I2C/flash/датчик) */
#define IMU_ACK_ERR_UNKNOWN 4u /* неизвестная команда */

/* --- Биты status в ORIENTATION --- */
#define IMU_STATUS_MPU_OK (1u << 0)
#define IMU_STATUS_MAG_OK (1u << 1)
#define IMU_STATUS_MAG_CAL (1u << 2)
#define IMU_STATUS_GYRO_CAL (1u << 3)
#define IMU_STATUS_FUSED_9X (1u << 4) /* 1 = 9-осевой режим, 0 = 6-осевой (без мага) */

/* --- Состояния калибровки (calib_state) --- */
#define IMU_CALIB_IDLE 0u
#define IMU_CALIB_MAG_RUN 1u
#define IMU_CALIB_GYRO_RUN 2u

/* Раскладка ORIENTATION (см. шапку файла). */
typedef struct {
    uint32_t ts_ms;
    float qw, qx, qy, qz; /* кватернион корпус->ENU (для sensor_msgs/Imu) */
    float roll_deg;       /* крен, -180..180 */
    float pitch_deg;      /* тангаж, -90..90 */
    float yaw_deg;        /* курс fused, -180..180, 0=север, + против часовой */
    float azimuth_deg;    /* азимут tilt-comp, 0..360, по часовой от севера, +склонение */
    float wx, wy, wz;     /* рад/с, со снятым дрейфом */
    float ax, ay, az;     /* м/с^2 */
    float mx, my, mz;     /* мкТл, калиброванные */
    float temp_c;
    uint8_t status;
    uint8_t calib_state;
    uint8_t rate_hz;
    uint8_t reserved;
} imu_orientation_t;

/* Раскладка INFO. */
typedef struct {
    uint8_t fw_major, fw_minor, fw_patch, reserved0;
    uint32_t uptime_ms;
    char board[16];
    uint8_t mpu_ok, mag_ok, mag_cal, gyro_cal;
    float declination_deg;
    uint8_t rate_hz;
    uint8_t reserved1[3];
} imu_info_t;

/* Раскладка ACK. */
typedef struct {
    uint8_t cmd_id;
    uint8_t result;
    uint16_t info; /* доп. информация (например, установленная частота) */
} imu_ack_t;

/* Раскладка CALIB. */
typedef struct {
    uint8_t state;
    uint8_t progress_pct;
    uint8_t reserved[2];
    float mag_hard_x, mag_hard_y, mag_hard_z;   /* hard-iron, мкТл */
    float mag_scale_x, mag_scale_y, mag_scale_z; /* soft-iron, безразм. */
    float gyro_bias_x, gyro_bias_y, gyro_bias_z; /* рад/с */
} imu_calib_msg_t;

/* CRC16-CCITT (poly 0x1021, init 0xFFFF). Проверка: "123456789" -> 0x29B1. */
uint16_t imu_crc16_ccitt(const uint8_t *data, size_t len);

/* Упаковать кадр. Возвращает длину кадра или 0, если out_cap мал. */
size_t imu_proto_encode(uint8_t msg_id, const uint8_t *payload, uint8_t payload_len,
                        uint8_t *out, size_t out_cap);

/* Сериализация структур в payload (буферы должны быть нужной длины). */
void imu_orientation_encode(const imu_orientation_t *m, uint8_t *out);
void imu_info_encode(const imu_info_t *m, uint8_t *out);
void imu_ack_encode(const imu_ack_t *m, uint8_t *out);
void imu_calib_msg_encode(const imu_calib_msg_t *m, uint8_t *out);

/* Десериализация (возвращают false при неверной длине). */
bool imu_orientation_decode(const uint8_t *in, uint8_t len, imu_orientation_t *m);
bool imu_ack_decode(const uint8_t *in, uint8_t len, imu_ack_t *m);

/* Потоковый декодер кадров. */
typedef struct {
    uint8_t state;
    uint8_t len; /* LEN поле кадра */
    uint8_t id;
    uint8_t idx;
    uint16_t crc_rx;
    uint8_t payload[IMU_PROTO_MAX_PAYLOAD];
} imu_decoder_t;

void imu_decoder_init(imu_decoder_t *d);
/* Скормить один байт. Возвращает true, когда принят целый кадр с верным CRC.
 * msg_id/len указывают внутрь декодера (копируйте сразу). */
bool imu_decoder_feed(imu_decoder_t *d, uint8_t byte,
                      uint8_t *msg_id, const uint8_t **payload, uint8_t *payload_len);

/* LE-хелперы. */
void imu_put_u16le(uint8_t *p, uint16_t v);
void imu_put_u32le(uint8_t *p, uint32_t v);
void imu_put_f32le(uint8_t *p, float v);
uint16_t imu_get_u16le(const uint8_t *p);
uint32_t imu_get_u32le(const uint8_t *p);
float imu_get_f32le(const uint8_t *p);

#ifdef __cplusplus
}
#endif

#endif /* IMU_PROTOCOL_H */
