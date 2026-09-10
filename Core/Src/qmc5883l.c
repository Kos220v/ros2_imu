#include "qmc5883l.h"

#include "imu_config.h"

/* Регистры QMC5883L */
#define QMC_REG_DATA_X_LSB 0x00u
#define QMC_REG_STATUS 0x06u
#define QMC_REG_CTRL1 0x09u
#define QMC_REG_CTRL2 0x0Au
#define QMC_REG_SET_RESET 0x0Bu
#define QMC_REG_CHIP_ID 0x0Du

#define QMC_STATUS_DRDY (1u << 0)
#define QMC_STATUS_OVL (1u << 1)

/* CTRL1: OSR=512, RNG=8G, ODR=100 Гц, MODE=continuous */
#define QMC_CTRL1_VAL 0x19u

static HAL_StatusTypeDef qmc_write(qmc5883l_t *dev, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(dev->hi2c, dev->dev_addr, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, IMU_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef qmc5883l_init(qmc5883l_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    dev->hi2c = hi2c;
    dev->dev_addr = (uint16_t)((uint16_t)addr7 << 1);
    dev->lsb_per_ut = 30.0f; /* 8G: 3000 LSB/Gauss = 30 LSB/мкТл */
    dev->ok = 0;

    if (HAL_I2C_IsDeviceReady(hi2c, dev->dev_addr, 3, IMU_I2C_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }

    uint8_t chip_id = 0;
    if (HAL_I2C_Mem_Read(hi2c, dev->dev_addr, QMC_REG_CHIP_ID, I2C_MEMADD_SIZE_8BIT,
                         &chip_id, 1, IMU_I2C_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }
    if (chip_id != QMC5883L_CHIP_ID_VAL) {
        return HAL_ERROR;
    }

    /* Программный сброс */
    if (qmc_write(dev, QMC_REG_CTRL2, 0x80u) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(10);
    /* Период SET/RESET */
    if (qmc_write(dev, QMC_REG_SET_RESET, 0x01u) != HAL_OK) {
        return HAL_ERROR;
    }
    if (qmc_write(dev, QMC_REG_CTRL2, 0x00u) != HAL_OK) {
        return HAL_ERROR;
    }
    /* Непрерывный режим */
    if (qmc_write(dev, QMC_REG_CTRL1, QMC_CTRL1_VAL) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(10);

    dev->ok = 1;
    return HAL_OK;
}

HAL_StatusTypeDef qmc5883l_read(qmc5883l_t *dev, float *mx, float *my, float *mz)
{
    uint8_t status = 0;
    if (HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr, QMC_REG_STATUS, I2C_MEMADD_SIZE_8BIT,
                         &status, 1, IMU_I2C_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }
    if (!(status & QMC_STATUS_DRDY)) {
        return HAL_BUSY; /* свежих данных нет - вызывающий оставит прошлое значение */
    }
    if (status & QMC_STATUS_OVL) {
        /* Переполнение: сбросить чтением данных и сообщить об ошибке */
        uint8_t dummy[6];
        (void)HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr, QMC_REG_DATA_X_LSB,
                               I2C_MEMADD_SIZE_8BIT, dummy, sizeof(dummy),
                               IMU_I2C_TIMEOUT_MS);
        return HAL_ERROR;
    }

    uint8_t raw[6];
    if (HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr, QMC_REG_DATA_X_LSB,
                         I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw),
                         IMU_I2C_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }

    int16_t xr = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
    int16_t yr = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
    int16_t zr = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));

    *mx = (float)xr / dev->lsb_per_ut;
    *my = (float)yr / dev->lsb_per_ut;
    *mz = (float)zr / dev->lsb_per_ut;
    return HAL_OK;
}
