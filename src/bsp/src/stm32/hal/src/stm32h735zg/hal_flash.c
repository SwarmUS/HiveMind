#include "hal/hal_flash.h"

bool Flash_program(uint32_t address, uint8_t* data, uint32_t bytesLength) {
    if (data == NULL) {
        return false;
    }

    HAL_FLASH_Unlock();

    // Round up to the nearest multiple of 8 bytes as the write works on 64 bit words.
    // If we are saving less than 64 bits, we will write garbage to the Flash, but it doesn't really
    // matter. The readback is done with a memcpy and won't read the garbage.
    uint32_t flashWordLength =
        (bytesLength + ((sizeof(uint32_t) * FLASH_NB_32BITWORD_IN_FLASHWORD) - 1)) /
        (sizeof(uint32_t) * FLASH_NB_32BITWORD_IN_FLASHWORD);

    for (unsigned int i = 0; i < flashWordLength; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, (uint32_t)data) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }

        address += (sizeof(uint32_t) * FLASH_NB_32BITWORD_IN_FLASHWORD);
        data += (sizeof(uint32_t) * FLASH_NB_32BITWORD_IN_FLASHWORD);
    }

    HAL_FLASH_Lock();
    return true;
}
