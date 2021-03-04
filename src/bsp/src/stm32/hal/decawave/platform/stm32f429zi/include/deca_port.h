#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hivemind_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

/**
 * @brief Performs a hardware reset on decawave
 */
void deca_hardwareReset();

/**
 * @brief Performs a wakeup on decawave using the SS line
 */
void deca_hardwareWakeup();

/**
 * @brief Sets decawave SPI to slow rate (used before deca PLL locks)
 */
void deca_setSlowRate();

/**
 * @brief Sets decawave SPI to fast rate (used once PLL has locked)
 */
void deca_setFastRate();

/**
 * @brief Initialises the decawave hardware
 */
void deca_init();

#ifdef __cplusplus
}
#endif

#endif // PORT_H