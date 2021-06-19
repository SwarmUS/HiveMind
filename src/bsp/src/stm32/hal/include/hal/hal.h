#ifndef __HAL_H__
#define __HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief  Initializes LL drivers for all used peripherals
 */
void Hal_init();

/**
 * @brief Powers on and configures all external peripherals used
 */
void Hal_bootup();

/**
 * @brief Calculates the 32bit CRC of the data
 * @param buffer Pointer to data
 * @param length Length of data in bytes
 * @return CRC32
 */
uint32_t Hal_calculateCRC32(const uint8_t* buffer, uint32_t length);

/**
 * @brief Generates a random number
 * @return a random number on 32 bits
 */
uint32_t Hal_generateRandomNumber();

#ifdef __cplusplus
}
#endif

#endif //__HAL_H__
