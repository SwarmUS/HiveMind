#include "hal/hal_flash.h"

bool Flash_program(uint32_t address, uint8_t* data, uint32_t bytesLength) {
    if (data == NULL) {
        return false;
    }

    HAL_FLASH_Unlock();

    for (unsigned int i = 0; i < bytesLength; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address, (uint64_t)(*data)) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }

        address += 1;
        data += 1;
    }

    HAL_FLASH_Lock();

    return true;
}
