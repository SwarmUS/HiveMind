#include "hal/hal_flash.h"

bool Flash_program(uint32_t address, uint8_t* data, uint32_t bytesLength) {
    if (data == NULL) {
        return false;
    }

    HAL_FLASH_Unlock();

    int32_t bytesLeft = (int32_t)bytesLength;
    while(bytesLeft > 0){
        HAL_StatusTypeDef ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, (uint32_t)data);
        if (ret != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        
        address += (8*FLASH_NB_32BITWORD_IN_FLASHWORD);
        data += FLASH_NB_32BITWORD_IN_FLASHWORD;
        bytesLeft -= ((uint16_t)FLASH_NB_32BITWORD_IN_FLASHWORD);

        while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_QW_BANK1)){}
    }

    HAL_FLASH_Lock();
    return true;
}
