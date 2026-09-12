#include "qmc5883l.h"

#include "imu_config.h"

/* ---------- QMC5883L (QST; HA5883/DB5883), datasheet rev 1.0 ---------- */
#define QMC_REG_DATA_X_LSB 0x00u /* 00H-05H: X_L,X_H,Y_L,Y_H,Z_L,Z_H */
#define QMC_REG_STATUS 0x06u     /* bit0 DRDY, bit1 OVL, bit2 DOR */
#define QMC_REG_CTRL1 0x09u      /* OSR | RNG | ODR | MODE */
#define QMC_REG_CTRL2 0x0Au      /* bit7 SOFT_RST, bit0 INT_ENB */
#define QMC_REG_SET_RESET 0x0Bu  /* FBR, рекомендуется 0x01 */
#define QMC_REG_CHIP_ID 0x0Du    /* возвращает 0xFF */

#define QMC_STATUS_DRDY (1u << 0)
#define QMC_STATUS_OVL (1u << 1)
#define QMC_CHIP_ID_VAL 0xFFu

/* CTRL1: OSR=512(00), RNG=8G(01), ODR=100 Гц(10), MODE=continuous(01) */
#define QMC_CTRL1_VAL 0x19u

/* 8G: 3000 LSB/Гauss = 30 LSB/мкТл */
#define QMC_LSB_PER_UT 30.0f

/* ---------- HMC5883L (Honeywell), datasheet 2014 ---------- */
#define HMC_REG_CRA 0x00u        /* MA(averaging) | DO(rate) | MS(mode) */
#define HMC_REG_CRB 0x01u        /* GN[2:0] gain */
#define HMC_REG_MODE 0x02u       /* MD[1:0]: 00 = continuous */
#define HMC_REG_DATA_X_MSB 0x03u /* 03H-08H: X_H,X_L,Z_H,Z_L,Y_H,Y_L */
#define HMC_REG_STATUS 0x09u
#define HMC_REG_ID_A 0x0Au       /* 'H' */
#define HMC_REG_ID_B 0x0Bu       /* '4' */
#define HMC_REG_ID_C 0x0Cu       /* '3' */

#define HMC_ID_A_VAL 0x48u
#define HMC_ID_B_VAL 0x34u
#define HMC_ID_C_VAL 0x33u

/* CRA: MA=8x(11), DO=75 Гц(110), MS=normal(00) */
#define HMC_CRA_VAL 0x7Cu
/* CRB: GN=111 -> ±8.1 G, 230 LSB/Gauss */
#define HMC_CRB_VAL 0xE0u
/* MODE: continuous */
#define HMC_MODE_VAL 0x00u

/* ±8.1G: 230 LSB/Гauss = 2.3 LSB/мкТл */
#define HMC_LSB_PER_UT 2.299f

/* ---------- Общие операции ---------- */

static HAL_StatusTypeDef mag_mem_read(qmc5883l_t *dev, uint8_t reg, uint8_t *buf,
                                      uint16_t len)
{
    return HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, IMU_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef mag_mem_write(qmc5883l_t *dev, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(dev->hi2c, dev->dev_addr, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, IMU_I2C_TIMEOUT_MS);
}

/* ---------- Определение QMC5883L ---------- */

static int probe_qmc(qmc5883l_t *dev, uint8_t addr7)
{
    dev->dev_addr = (uint16_t)addr7 << 1;
    if (HAL_I2C_IsDeviceReady(dev->hi2c, dev->dev_addr, 3, IMU_I2C_TIMEOUT_MS) != HAL_OK) {
        return 0;
    }
    uint8_t id = 0;
    if (mag_mem_read(dev, QMC_REG_CHIP_ID, &id, 1) != HAL_OK) {
        return 0;
    }
    if (id != QMC_CHIP_ID_VAL) {
        return 0;
    }

    /* Soft reset (CTRL2.SOFT_RST), период SET/RESET, непрерывный режим */
    if (mag_mem_write(dev, QMC_REG_CTRL2, 0x80u) != HAL_OK) {
        return 0;
    }
    HAL_Delay(10);
    if (mag_mem_write(dev, QMC_REG_SET_RESET, 0x01u) != HAL_OK) {
        return 0;
    }
    if (mag_mem_write(dev, QMC_REG_CTRL2, 0x00u) != HAL_OK) {
        return 0;
    }
    if (mag_mem_write(dev, QMC_REG_CTRL1, QMC_CTRL1_VAL) != HAL_OK) {
        return 0;
    }
    HAL_Delay(10);

    dev->type = MAG5883_QMC;
    dev->lsb_per_ut = QMC_LSB_PER_UT;
    return 1;
}

/* ---------- Определение HMC5883L ---------- */

static int probe_hmc(qmc5883l_t *dev, uint8_t addr7)
{
    dev->dev_addr = (uint16_t)addr7 << 1;
    if (HAL_I2C_IsDeviceReady(dev->hi2c, dev->dev_addr, 3, IMU_I2C_TIMEOUT_MS) != HAL_OK) {
        return 0;
    }
    uint8_t id_a = 0, id_b = 0, id_c = 0;
    if (mag_mem_read(dev, HMC_REG_ID_A, &id_a, 1) != HAL_OK) {
        return 0;
    }
    if (mag_mem_read(dev, HMC_REG_ID_B, &id_b, 1) != HAL_OK) {
        return 0;
    }
    if (mag_mem_read(dev, HMC_REG_ID_C, &id_c, 1) != HAL_OK) {
        return 0;
    }
    if (id_a != HMC_ID_A_VAL || id_b != HMC_ID_B_VAL || id_c != HMC_ID_C_VAL) {
        return 0;
    }

    /* Среднее 8x, 75 Гц, усиление ±8.1 G, непрерывный режим */
    if (mag_mem_write(dev, HMC_REG_CRA, HMC_CRA_VAL) != HAL_OK) {
        return 0;
    }
    if (mag_mem_write(dev, HMC_REG_CRB, HMC_CRB_VAL) != HAL_OK) {
        return 0;
    }
    if (mag_mem_write(dev, HMC_REG_MODE, HMC_MODE_VAL) != HAL_OK) {
        return 0;
    }
    HAL_Delay(20); /* первому измерению после смены усиления нужна полная цикличность */

    dev->type = MAG5883_HMC;
    dev->lsb_per_ut = HMC_LSB_PER_UT;
    return 1;
}

/* ---------- Публичный API ---------- */

HAL_StatusTypeDef qmc5883l_init(qmc5883l_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    dev->hi2c = hi2c;
    dev->ok = 0;
    dev->type = MAG5883_QMC;
    dev->lsb_per_ut = QMC_LSB_PER_UT;

    /* Сначала указанный адрес (оба варианта), затем другой стандартный */
    if (probe_qmc(dev, addr7) || probe_hmc(dev, addr7)) {
        dev->ok = 1;
        return HAL_OK;
    }
    uint8_t alt = (addr7 == QMC5883L_ADDR7) ? HMC5883L_ADDR7 : QMC5883L_ADDR7;
    if (probe_qmc(dev, alt) || probe_hmc(dev, alt)) {
        dev->ok = 1;
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef qmc5883l_read(qmc5883l_t *dev, float *mx, float *my, float *mz)
{
    uint8_t raw[6];
    int16_t xr, yr, zr;

    if (dev->type == MAG5883_QMC) {
        uint8_t status = 0;
        if (mag_mem_read(dev, QMC_REG_STATUS, &status, 1) != HAL_OK) {
            return HAL_ERROR;
        }
        if (!(status & QMC_STATUS_DRDY)) {
            return HAL_BUSY; /* свежих данных нет — вызывающий оставит прошлое значение */
        }
        if (status & QMC_STATUS_OVL) {
            /* Переполнение: прочесть данные (сбрасывает флаг) и сообщить об ошибке */
            (void)mag_mem_read(dev, QMC_REG_DATA_X_LSB, raw, sizeof(raw));
            return HAL_ERROR;
        }
        if (mag_mem_read(dev, QMC_REG_DATA_X_LSB, raw, sizeof(raw)) != HAL_OK) {
            return HAL_ERROR;
        }
        /* 00H-05H: X_L, X_H, Y_L, Y_H, Z_L, Z_H */
        xr = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
        yr = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
        zr = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));
    } else {
        /* HMC: 03H-08H: X_H, X_L, Z_H, Z_L, Y_H, Y_L (MSB first!) */
        if (mag_mem_read(dev, HMC_REG_DATA_X_MSB, raw, sizeof(raw)) != HAL_OK) {
            return HAL_ERROR;
        }
        xr = (int16_t)((uint16_t)((uint16_t)raw[0] << 8) | (uint16_t)raw[1]);
        zr = (int16_t)((uint16_t)((uint16_t)raw[2] << 8) | (uint16_t)raw[3]);
        yr = (int16_t)((uint16_t)((uint16_t)raw[4] << 8) | (uint16_t)raw[5]);
    }

    *mx = (float)xr / dev->lsb_per_ut;
    *my = (float)yr / dev->lsb_per_ut;
    *mz = (float)zr / dev->lsb_per_ut;
    return HAL_OK;
}
