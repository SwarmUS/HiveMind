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

bool Flash_program(uint32_t address, uint8_t* data, uint32_t size) {
    if (data == NULL) {
        return false;
    }

    HAL_FLASH_Unlock();

    for (unsigned int i = 0; i < size; i += 4) {
        if (HAL_FLASH_Program(FLASH_PROGRAM_32_BITS, address, (uint64_t)(*data)) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }

        address += 4;
        data += 4;
    }

    HAL_FLASH_Lock();

    return true;
}
