#include "imu_protocol.h"

#include <string.h>

uint16_t imu_crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

size_t imu_proto_encode(uint8_t msg_id, const uint8_t *payload, uint8_t payload_len,
                        uint8_t *out, size_t out_cap)
{
    if (payload_len > IMU_PROTO_MAX_PAYLOAD) {
        return 0;
    }
    size_t need = (size_t)payload_len + 6u;
    if (out_cap < need) {
        return 0;
    }
    out[0] = IMU_PROTO_SYNC0;
    out[1] = IMU_PROTO_SYNC1;
    out[2] = (uint8_t)(payload_len + 1u);
    out[3] = msg_id;
    if (payload_len > 0) {
        memcpy(&out[4], payload, payload_len);
    }
    uint16_t crc = imu_crc16_ccitt(&out[2], (size_t)payload_len + 2u);
    out[4 + payload_len] = (uint8_t)(crc & 0xFFu);
    out[5 + payload_len] = (uint8_t)(crc >> 8);
    return need;
}

void imu_put_u16le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

void imu_put_u32le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)(v >> 24);
}

void imu_put_f32le(uint8_t *p, float v)
{
    uint32_t u;
    memcpy(&u, &v, sizeof(u));
    imu_put_u32le(p, u);
}

uint16_t imu_get_u16le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint32_t imu_get_u32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

float imu_get_f32le(const uint8_t *p)
{
    uint32_t u = imu_get_u32le(p);
    float v;
    memcpy(&v, &u, sizeof(v));
    return v;
}

void imu_orientation_encode(const imu_orientation_t *m, uint8_t *out)
{
    imu_put_u32le(&out[0], m->ts_ms);
    imu_put_f32le(&out[4], m->qw);
    imu_put_f32le(&out[8], m->qx);
    imu_put_f32le(&out[12], m->qy);
    imu_put_f32le(&out[16], m->qz);
    imu_put_f32le(&out[20], m->roll_deg);
    imu_put_f32le(&out[24], m->pitch_deg);
    imu_put_f32le(&out[28], m->yaw_deg);
    imu_put_f32le(&out[32], m->azimuth_deg);
    imu_put_f32le(&out[36], m->wx);
    imu_put_f32le(&out[40], m->wy);
    imu_put_f32le(&out[44], m->wz);
    imu_put_f32le(&out[48], m->ax);
    imu_put_f32le(&out[52], m->ay);
    imu_put_f32le(&out[56], m->az);
    imu_put_f32le(&out[60], m->mx);
    imu_put_f32le(&out[64], m->my);
    imu_put_f32le(&out[68], m->mz);
    imu_put_f32le(&out[72], m->temp_c);
    out[76] = m->status;
    out[77] = m->calib_state;
    out[78] = m->rate_hz;
    out[79] = m->reserved;
}

bool imu_orientation_decode(const uint8_t *in, uint8_t len, imu_orientation_t *m)
{
    if (len != IMU_ORIENTATION_LEN) {
        return false;
    }
    m->ts_ms = imu_get_u32le(&in[0]);
    m->qw = imu_get_f32le(&in[4]);
    m->qx = imu_get_f32le(&in[8]);
    m->qy = imu_get_f32le(&in[12]);
    m->qz = imu_get_f32le(&in[16]);
    m->roll_deg = imu_get_f32le(&in[20]);
    m->pitch_deg = imu_get_f32le(&in[24]);
    m->yaw_deg = imu_get_f32le(&in[28]);
    m->azimuth_deg = imu_get_f32le(&in[32]);
    m->wx = imu_get_f32le(&in[36]);
    m->wy = imu_get_f32le(&in[40]);
    m->wz = imu_get_f32le(&in[44]);
    m->ax = imu_get_f32le(&in[48]);
    m->ay = imu_get_f32le(&in[52]);
    m->az = imu_get_f32le(&in[56]);
    m->mx = imu_get_f32le(&in[60]);
    m->my = imu_get_f32le(&in[64]);
    m->mz = imu_get_f32le(&in[68]);
    m->temp_c = imu_get_f32le(&in[72]);
    m->status = in[76];
    m->calib_state = in[77];
    m->rate_hz = in[78];
    m->reserved = in[79];
    return true;
}

void imu_info_encode(const imu_info_t *m, uint8_t *out)
{
    out[0] = m->fw_major;
    out[1] = m->fw_minor;
    out[2] = m->fw_patch;
    out[3] = m->reserved0;
    imu_put_u32le(&out[4], m->uptime_ms);
    memcpy(&out[8], m->board, 16);
    out[24] = m->mpu_ok;
    out[25] = m->mag_ok;
    out[26] = m->mag_cal;
    out[27] = m->gyro_cal;
    imu_put_f32le(&out[28], m->declination_deg);
    out[32] = m->rate_hz;
    out[33] = m->reserved1[0];
    out[34] = m->reserved1[1];
    out[35] = m->reserved1[2];
}

void imu_ack_encode(const imu_ack_t *m, uint8_t *out)
{
    out[0] = m->cmd_id;
    out[1] = m->result;
    imu_put_u16le(&out[2], m->info);
}

bool imu_ack_decode(const uint8_t *in, uint8_t len, imu_ack_t *m)
{
    if (len != IMU_ACK_LEN) {
        return false;
    }
    m->cmd_id = in[0];
    m->result = in[1];
    m->info = imu_get_u16le(&in[2]);
    return true;
}

void imu_calib_msg_encode(const imu_calib_msg_t *m, uint8_t *out)
{
    out[0] = m->state;
    out[1] = m->progress_pct;
    out[2] = m->reserved[0];
    out[3] = m->reserved[1];
    imu_put_f32le(&out[4], m->mag_hard_x);
    imu_put_f32le(&out[8], m->mag_hard_y);
    imu_put_f32le(&out[12], m->mag_hard_z);
    imu_put_f32le(&out[16], m->mag_scale_x);
    imu_put_f32le(&out[20], m->mag_scale_y);
    imu_put_f32le(&out[24], m->mag_scale_z);
    imu_put_f32le(&out[28], m->gyro_bias_x);
    imu_put_f32le(&out[32], m->gyro_bias_y);
    imu_put_f32le(&out[36], m->gyro_bias_z);
}

/* Состояния декодера. */
enum {
    DEC_SYNC0 = 0,
    DEC_SYNC1,
    DEC_LEN,
    DEC_ID,
    DEC_PAYLOAD,
    DEC_CRC_LO,
    DEC_CRC_HI
};

void imu_decoder_init(imu_decoder_t *d)
{
    memset(d, 0, sizeof(*d));
    d->state = DEC_SYNC0;
}

static void imu_decoder_reset(imu_decoder_t *d)
{
    d->state = DEC_SYNC0;
    d->len = 0;
    d->id = 0;
    d->idx = 0;
    d->crc_rx = 0;
}

bool imu_decoder_feed(imu_decoder_t *d, uint8_t byte,
                      uint8_t *msg_id, const uint8_t **payload, uint8_t *payload_len)
{
    switch (d->state) {
    case DEC_SYNC0:
        if (byte == IMU_PROTO_SYNC0) {
            d->state = DEC_SYNC1;
        }
        break;
    case DEC_SYNC1:
        if (byte == IMU_PROTO_SYNC1) {
            d->state = DEC_LEN;
        } else if (byte == IMU_PROTO_SYNC0) {
            d->state = DEC_SYNC1; /* повторный SYNC0 */
        } else {
            imu_decoder_reset(d);
        }
        break;
    case DEC_LEN:
        if (byte < 1u || byte > (IMU_PROTO_MAX_PAYLOAD + 1u)) {
            /* мусор: возможно, это начало нового кадра */
            if (byte == IMU_PROTO_SYNC0) {
                d->state = DEC_SYNC1;
            } else {
                imu_decoder_reset(d);
            }
        } else {
            d->len = byte;
            d->state = DEC_ID;
        }
        break;
    case DEC_ID:
        d->id = byte;
        d->idx = 0;
        d->state = (d->len > 1u) ? DEC_PAYLOAD : DEC_CRC_LO;
        break;
    case DEC_PAYLOAD:
        d->payload[d->idx++] = byte;
        if (d->idx >= (uint8_t)(d->len - 1u)) {
            d->state = DEC_CRC_LO;
        }
        break;
    case DEC_CRC_LO:
        d->crc_rx = byte;
        d->state = DEC_CRC_HI;
        break;
    case DEC_CRC_HI: {
        d->crc_rx |= (uint16_t)((uint16_t)byte << 8);
        /* CRC считается от LEN, ID и PAYLOAD */
        uint8_t tmp[2 + IMU_PROTO_MAX_PAYLOAD];
        tmp[0] = d->len;
        tmp[1] = d->id;
        if (d->len > 1u) {
            memcpy(&tmp[2], d->payload, (size_t)d->len - 1u);
        }
        uint16_t crc = imu_crc16_ccitt(tmp, (size_t)d->len + 1u);
        bool ok = (crc == d->crc_rx);
        uint8_t id = d->id;
        uint8_t plen = (uint8_t)(d->len - 1u);
        imu_decoder_reset(d);
        if (ok) {
            *msg_id = id;
            *payload = d->payload;
            *payload_len = plen;
            return true;
        }
        /* неверный CRC: если этот байт похож на SYNC0, не теряем его */
        if (byte == IMU_PROTO_SYNC0) {
            d->state = DEC_SYNC1;
        }
        break;
    }
    default:
        imu_decoder_reset(d);
        break;
    }
    return false;
}
