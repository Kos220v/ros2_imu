/* Реализация стаба HAL для хост-тестов. */
#include "stm32f3xx_hal.h"

#include <string.h>

#define STUB_TX_CAP (1u << 20)
#define STUB_I2C_LOG 64u

static uint32_t s_tick;
static UART_HandleTypeDef *s_data_huart;
static uint8_t s_tx[STUB_TX_CAP];
static size_t s_tx_len;

static UART_HandleTypeDef *s_rx_huart;
static uint8_t *s_rx_ptr;
static uint16_t s_rx_size;

static int s_mpu_present = 1;
static int s_mag_present = 1;
static int16_t s_mpu_raw[7]; /* ax ay az t gx gy gz */
static int16_t s_mag_raw[3];
static uint8_t s_mag_status = 0x01; /* DRDY */

static stub_i2c_write_t s_writes[STUB_I2C_LOG];
static int s_write_count;

static uint8_t s_flash[2048];

/* Адреса на шине (8-битные, как их видит HAL) */
#define STUB_MPU_ADDR 0xD0u
#define STUB_MAG_ADDR 0x1Au

void stub_reset(void)
{
    s_tick = 0;
    s_tx_len = 0;
    s_rx_huart = NULL;
    s_rx_ptr = NULL;
    s_rx_size = 0;
    s_mpu_present = 1;
    s_mag_present = 1;
    s_mag_status = 0x01;
    s_write_count = 0;
    memset(s_writes, 0, sizeof(s_writes));
    memset(s_flash, 0xFF, sizeof(s_flash));
    /* Плата горизонтальна, смотрит на север, гироскоп стоит */
    s_mpu_raw[0] = 0;
    s_mpu_raw[1] = 0;
    s_mpu_raw[2] = 8192; /* 1 g при диапазоне 4 g */
    s_mpu_raw[3] = 0;
    s_mpu_raw[4] = 0;
    s_mpu_raw[5] = 0;
    s_mpu_raw[6] = 0;
    s_mag_raw[0] = 600;   /* 20 мкТл */
    s_mag_raw[1] = 0;
    s_mag_raw[2] = -1350; /* -45 мкТл */
}

void stub_tick_set(uint32_t t)
{
    s_tick = t;
}

void stub_tick_advance(uint32_t ms)
{
    s_tick += ms;
}

uint32_t HAL_GetTick(void)
{
    return s_tick;
}

void HAL_Delay(uint32_t ms)
{
    (void)ms;
}

/* ---------- I2C ---------- */

void stub_i2c_set_present(int mpu, int mag)
{
    s_mpu_present = mpu;
    s_mag_present = mag;
}

void stub_mpu_set_raw(int16_t ax, int16_t ay, int16_t az, int16_t t,
                      int16_t gx, int16_t gy, int16_t gz)
{
    s_mpu_raw[0] = ax;
    s_mpu_raw[1] = ay;
    s_mpu_raw[2] = az;
    s_mpu_raw[3] = t;
    s_mpu_raw[4] = gx;
    s_mpu_raw[5] = gy;
    s_mpu_raw[6] = gz;
}

void stub_mag_set_raw(int16_t x, int16_t y, int16_t z)
{
    s_mag_raw[0] = x;
    s_mag_raw[1] = y;
    s_mag_raw[2] = z;
}

void stub_mag_set_status(uint8_t status)
{
    s_mag_status = status;
}

int stub_i2c_write_count(void)
{
    return s_write_count;
}

const stub_i2c_write_t *stub_i2c_writes(void)
{
    return s_writes;
}

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                        uint32_t Trials, uint32_t Timeout)
{
    (void)hi2c;
    (void)Trials;
    (void)Timeout;
    if (DevAddress == STUB_MPU_ADDR) {
        return s_mpu_present ? HAL_OK : HAL_ERROR;
    }
    if (DevAddress == STUB_MAG_ADDR) {
        return s_mag_present ? HAL_OK : HAL_ERROR;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                    uint16_t MemAddress, uint16_t MemAddSize,
                                    uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    (void)hi2c;
    (void)MemAddSize;
    (void)Timeout;
    if (HAL_I2C_IsDeviceReady(hi2c, DevAddress, 1, 10) != HAL_OK) {
        return HAL_ERROR;
    }
    if (Size >= 1 && s_write_count < (int)STUB_I2C_LOG) {
        s_writes[s_write_count].dev = DevAddress;
        s_writes[s_write_count].reg = (uint8_t)MemAddress;
        s_writes[s_write_count].val = pData[0];
        s_write_count++;
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                   uint16_t MemAddress, uint16_t MemAddSize,
                                   uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    (void)hi2c;
    (void)MemAddSize;
    (void)Timeout;
    if (DevAddress == STUB_MPU_ADDR && !s_mpu_present) {
        return HAL_ERROR;
    }
    if (DevAddress == STUB_MAG_ADDR && !s_mag_present) {
        return HAL_ERROR;
    }
    if (DevAddress == STUB_MPU_ADDR) {
        if (MemAddress == 0x75u && Size == 1) { /* WHO_AM_I */
            pData[0] = 0x68u;
            return HAL_OK;
        }
        if (MemAddress == 0x3Bu && Size == 14) {
            for (int i = 0; i < 7; i++) {
                pData[2 * i] = (uint8_t)(((uint16_t)s_mpu_raw[i] >> 8) & 0xFFu);
                pData[2 * i + 1] = (uint8_t)((uint16_t)s_mpu_raw[i] & 0xFFu);
            }
            return HAL_OK;
        }
    }
    if (DevAddress == STUB_MAG_ADDR) {
        if (MemAddress == 0x0Du && Size == 1) { /* CHIP ID */
            pData[0] = 0xFFu;
            return HAL_OK;
        }
        if (MemAddress == 0x06u && Size == 1) { /* STATUS */
            pData[0] = s_mag_status;
            return HAL_OK;
        }
        if (MemAddress == 0x00u && Size == 6) {
            for (int i = 0; i < 3; i++) {
                pData[2 * i] = (uint8_t)((uint16_t)s_mag_raw[i] & 0xFFu);
                pData[2 * i + 1] = (uint8_t)(((uint16_t)s_mag_raw[i] >> 8) & 0xFFu);
            }
            return HAL_OK;
        }
    }
    memset(pData, 0, Size);
    return HAL_OK;
}

/* ---------- UART ---------- */

void stub_uart_set_data_handle(UART_HandleTypeDef *huart)
{
    s_data_huart = huart;
}

const uint8_t *stub_uart_tx_data(void)
{
    return s_tx;
}

size_t stub_uart_tx_len(void)
{
    return s_tx_len;
}

void stub_uart_tx_clear(void)
{
    s_tx_len = 0;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData,
                                    uint16_t Size, uint32_t Timeout)
{
    (void)Timeout;
    if (huart != s_data_huart) {
        return HAL_OK; /* отладочный порт: игнорируем */
    }
    size_t room = (s_tx_len < STUB_TX_CAP) ? (STUB_TX_CAP - s_tx_len) : 0;
    size_t n = (Size < room) ? Size : room;
    memcpy(&s_tx[s_tx_len], pData, n);
    s_tx_len += n;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData,
                                      uint16_t Size)
{
    s_rx_huart = huart;
    s_rx_ptr = pData;
    s_rx_size = Size;
    return HAL_OK;
}

void stub_uart_inject_rx(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        if (!s_rx_ptr || s_rx_size < 1) {
            return;
        }
        *s_rx_ptr = data[i];
        /* Колбэк перевооружит приём (обновит s_rx_ptr) */
        HAL_UART_RxCpltCallback(s_rx_huart);
    }
}

/* ---------- FLASH ---------- */

uint8_t *hal_stub_flash_page(void)
{
    return s_flash;
}

HAL_StatusTypeDef HAL_FLASH_Unlock(void)
{
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASH_Lock(void)
{
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *PageError)
{
    (void)pEraseInit;
    if (PageError) {
        *PageError = 0xFFFFFFFFu;
    }
    memset(s_flash, 0xFF, sizeof(s_flash));
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
{
    (void)TypeProgram;
    /* Address на хосте - смещение от начала страницы (см. imu_flash.c) */
    if (Address + 1u >= sizeof(s_flash)) {
        return HAL_ERROR;
    }
    /* Эмуляция flash: биты только сбрасываются */
    s_flash[Address] = (uint8_t)(s_flash[Address] & (Data & 0xFFu));
    s_flash[Address + 1] = (uint8_t)(s_flash[Address + 1] & ((Data >> 8) & 0xFFu));
    return HAL_OK;
}
