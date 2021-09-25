#include "hal/hal_flash.h"

bool Flash_eraseSector(uint8_t sector) {
    FLASH_EraseInitTypeDef eraseConfig = {.TypeErase = FLASH_TYPEERASE_SECTORS,
                                          .Sector = sector,
                                          .NbSectors = 1,
                                          .VoltageRange = FLASH_VOLTAGE_RANGE_3};
    uint32_t badSector = 0;

    HAL_FLASH_Unlock();
    bool ret = HAL_FLASHEx_Erase(&eraseConfig, &badSector) == HAL_OK;
    HAL_FLASH_Lock();

    return ret;
}