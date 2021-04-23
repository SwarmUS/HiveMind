#ifndef __HAL_FLASH_H__
#define __HAL_FLASH_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "hivemind_hal.h"

/**
 * @brief Erases a sector form flash
 * @param sector Sector to erase
 * @return True if successful, false otherwise
 */
bool Flash_eraseSector(uint8_t sector);

/**
 * @brief Saves data to previously erased flash
 * @param address Address at which to begin writing
 * @param data Pointer to the data to write
 * @param size Size of the data (in bytes)
 * @return True if successful, false otherwise
 */
bool Flash_program(uint32_t address, uint8_t* data, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif //__HAL_FLASH_H__
