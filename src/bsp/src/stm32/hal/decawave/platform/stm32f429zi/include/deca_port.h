#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hivemind_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

/**
 * @brief Enum to specify which decawave we are addressing
 */
typedef enum { DW_A = 0, DW_B } decaDevice_t;

/**
 * @brief Structure to hold a port and pin associated with a decawave NSS pin
 */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} decaNSSConfig_t;

/**
 * @brief Performs a hardware reset on a specific decawave
 * @param selectedDevice The device to reset
 */
void deca_hardwareReset(decaDevice_t selectedDevice);

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

/**
 * @brief Choose which decawave is selected for the next SPI communication
 * @param selectedDevice The device to select
 */
void deca_selectDevice(decaDevice_t selectedDevice);

/**
 * @brief Gets the NSS configuration for the currently selected decawave
 * @return The NSS pin
 */
decaNSSConfig_t* deca_getSelectedNSSConfig();

#ifdef __cplusplus
}
#endif

#endif // PORT_H
