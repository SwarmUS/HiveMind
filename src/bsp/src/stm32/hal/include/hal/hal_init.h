#ifndef __HAL_INIT_H__
#define __HAL_INIT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief Initializes board specific HAL peripherals
 */
void Hal_initPlatformSpecific();

/**
 * @brief Tells if the WROOM has power running to it
 * @return True if powered
 */
bool Hal_wroomPowerEnabled();

/**
 * @brief Enables the WROOM by taking it out of reset
 */
void Hal_enableWroom();

/**
 * @brief Tells if the PHY has power running to it
 * @return True if powered
 */
bool Hal_ethernetPowerEnabled();

/**
 * @brief Enables the PHY by taking it out of reset
 */
void Hal_enableEthernet();

#ifdef __cplusplus
}
#endif

#endif //__HAL_INIT_H__
