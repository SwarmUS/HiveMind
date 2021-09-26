#include "hal/hal_flash.h"

bool Flash_program(uint32_t address, uint8_t* data, uint32_t bytesLength) {
    SCB_InvalidateDCache();
    if (data == NULL) {
        return false;
    }

    HAL_FLASH_Unlock();

    // Round up to the nearest multiple of 8 bytes as the write works on 64 bit words.
    // If we are saving less than 64 bits, we will write garbage to the Flash, but it doesn't really
    // matter. The readback is done with a memcpy and won't read the garbage.
    uint32_t flashWordLength = (bytesLength + (sizeof(uint64_t) - 1)) / sizeof(uint64_t);

    for (unsigned int i = 0; i < flashWordLength; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, (uint32_t)data) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }

        address += sizeof(uint64_t);
        data += sizeof(uint64_t);
    }

    HAL_FLASH_Lock();

    SCB_InvalidateDCache();
    return true;
}
