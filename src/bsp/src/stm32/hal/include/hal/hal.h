#ifndef __HAL_H__
#define __HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief  Inits the hal of the device
 */
void Hal_init();

/**
 * @brief Calculates the 32bit CRC of the data
 * @param buffer Pointer to data
 * @param length Length of data in bytes
 * @return CRC32
 */
uint32_t Hal_calculateCRC32(const uint8_t* buffer, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif //__HAL_H__
