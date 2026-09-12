/* Минимальный стаб STM32 HAL для хост-тестов (gcc на ПК).
 * Подключается вместо настоящего stm32f3xx_hal.h через -I tests/host/hal_stub.
 * НЕ использовать в прошивке!
 */
#ifndef HAL_STUB_STM32F3XX_HAL_H
#define HAL_STUB_STM32F3XX_HAL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1,
    HAL_BUSY = 2,
    HAL_TIMEOUT = 3
} HAL_StatusTypeDef;

typedef struct {
    void *Instance;
} I2C_HandleTypeDef;

typedef struct {
    void *Instance;
} UART_HandleTypeDef;

#define I2C_MEMADD_SIZE_8BIT 1u

typedef struct {
    uint32_t TypeErase;
    uint32_t PageAddress;
    uint32_t NbPages;
} FLASH_EraseInitTypeDef;

#define FLASH_TYPEERASE_PAGES 0u
#define FLASH_TYPEPROGRAM_HALFWORD 0u

/* --- I2C --- */
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                   uint16_t MemAddress, uint16_t MemAddSize,
                                   uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                    uint16_t MemAddress, uint16_t MemAddSize,
                                    uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                        uint32_t Trials, uint32_t Timeout);

/* --- UART --- */
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData,
                                    uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData,
                                      uint16_t Size);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart); /* определён в imu_app.c */

/* --- FLASH --- */
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *PageError);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);

/* --- Прочее --- */
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);

#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)

/* --- Управление стабом (только для тестов) --- */
void stub_reset(void);
void stub_tick_set(uint32_t t);
void stub_tick_advance(uint32_t ms);

/* I2C: присутствие датчиков и сырые данные */
void stub_i2c_set_present(int mpu, int mag);
void stub_mpu_set_raw(int16_t ax, int16_t ay, int16_t az, int16_t t,
                      int16_t gx, int16_t gy, int16_t gz);
void stub_mag_set_raw(int16_t x, int16_t y, int16_t z);
void stub_mag_set_status(uint8_t status); /* DRDY/OVL биты */
void stub_mag_set_ut(float x, float y, float z);   /* истинное поле, мкТл */
void stub_mag_set_hmc(int enable);  /* эмулировать HMC5883L@0x1E вместо QMC */
void stub_mag_set_dual(int enable); /* QMC и HMC одновременно на шине */
typedef struct {
    uint16_t dev;
    uint8_t reg;
    uint8_t val;
} stub_i2c_write_t;
int stub_i2c_write_count(void);
const stub_i2c_write_t *stub_i2c_writes(void);

/* UART: какой хэндл считается "данными" (остальные игнорируются) */
void stub_uart_set_data_handle(UART_HandleTypeDef *huart);
const uint8_t *stub_uart_tx_data(void);
size_t stub_uart_tx_len(void);
void stub_uart_tx_clear(void);
/* Впрыснуть входящие байты (вызывает RxCpltCallback побайтно) */
void stub_uart_inject_rx(const uint8_t *data, size_t len);

/* FLASH: прямой доступ к фейковой странице */
uint8_t *hal_stub_flash_page(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_STUB_STM32F3XX_HAL_H */
