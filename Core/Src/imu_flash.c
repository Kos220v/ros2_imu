#include "imu_flash.h"

#include <string.h>

#include "imu_config.h"
#include "imu_protocol.h"
#include "stm32f3xx_hal.h"

#define IMU_FLASH_VERSION 1u

/* Раскладка записи во flash (36 байт). */
typedef struct {
    uint32_t magic;
    uint8_t version;
    uint8_t reserved;
    uint16_t len; /* длина imu_calib_t */
    imu_calib_t calib;
    uint16_t crc;
} imu_flash_record_t;

static uint8_t *imu_flash_page(void)
{
#ifdef IMU_HOST_TEST
    extern uint8_t *hal_stub_flash_page(void);
    return hal_stub_flash_page();
#else
    return (uint8_t *)IMU_FLASH_PAGE_ADDR;
#endif
}

static uint16_t imu_flash_record_crc(const imu_calib_t *calib)
{
    return imu_crc16_ccitt((const uint8_t *)calib, sizeof(imu_calib_t));
}

bool imu_flash_load(imu_calib_t *calib)
{
    const imu_flash_record_t *rec = (const imu_flash_record_t *)imu_flash_page();
    if (rec->magic != IMU_FLASH_MAGIC) {
        return false;
    }
    if (rec->version != IMU_FLASH_VERSION) {
        return false;
    }
    if (rec->len != sizeof(imu_calib_t)) {
        return false;
    }
    if (rec->crc != imu_flash_record_crc(&rec->calib)) {
        return false;
    }
    /* Проверка диапазонов */
    if (rec->calib.rate_hz != 10 && rec->calib.rate_hz != 25 &&
        rec->calib.rate_hz != 50 && rec->calib.rate_hz != 100) {
        return false;
    }
    if (rec->calib.declination_deg < -30.0f || rec->calib.declination_deg > 30.0f) {
        return false;
    }
    for (int i = 0; i < 3; i++) {
        if (rec->calib.mag_scale[i] < 0.3f || rec->calib.mag_scale[i] > 3.0f) {
            return false;
        }
    }
    *calib = rec->calib;
    return true;
}

bool imu_flash_save(const imu_calib_t *calib)
{
    imu_flash_record_t rec;
    rec.magic = IMU_FLASH_MAGIC;
    rec.version = IMU_FLASH_VERSION;
    rec.reserved = 0;
    rec.len = (uint16_t)sizeof(imu_calib_t);
    rec.calib = *calib;
    rec.crc = imu_flash_record_crc(calib);

    uint8_t *page = imu_flash_page();

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
#ifdef IMU_HOST_TEST
    erase.PageAddress = 0; /* стаб игнорирует адрес */
#else
    erase.PageAddress = IMU_FLASH_PAGE_ADDR;
#endif
    erase.NbPages = 1;
    uint32_t page_err = 0;
    bool ok = (HAL_FLASHEx_Erase(&erase, &page_err) == HAL_OK);

    /* Пишем полусловами */
    const uint8_t *src = (const uint8_t *)&rec;
    for (size_t off = 0; ok && off < sizeof(rec); off += 2) {
        uint16_t hw = (uint16_t)src[off] | (uint16_t)((uint16_t)src[off + 1] << 8);
#ifdef IMU_HOST_TEST
        /* На хосте стаб принимает смещение от начала страницы */
        uint32_t addr = (uint32_t)off;
        (void)page;
#else
        uint32_t addr = IMU_FLASH_PAGE_ADDR + (uint32_t)off;
#endif
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, (uint64_t)hw) != HAL_OK) {
            ok = false;
        }
    }

    HAL_FLASH_Lock();

    /* Проверка чтением */
    if (ok) {
        imu_calib_t check;
        ok = imu_flash_load(&check);
    }
    return ok;
}
