#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hivemind_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <deca_device_api.h>

typedef void (*decaISRCallback_t)(void* context);

/**
 * @brief Enum to specify which decawave we are addressing
 */
typedef enum { DW_A = 0, DW_B } decaDevice_t;

/**
 * @brief Structure to hold a port and pin associated with a decawave NSS pin
 */
typedef struct {
    GPIO_TypeDef* nssPort;
    uint16_t nssPin;
    GPIO_TypeDef* irqPort;
    uint16_t irqPin;
    uint8_t deviceIndex;
    void* isrContext;
    decaISRCallback_t isrCallback;
} decawaveDeviceConfig_t;

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
 * @brief Gets the device configuration for the currently selected decawave
 * @return The device configuration
 */
decawaveDeviceConfig_t* deca_getSelectedDevice();

/**
 * @brief Gets the NSS configuration for a given decawave
 * @return The NSS pin
 */
decawaveDeviceConfig_t* deca_getDevice(decaDevice_t device);

void deca_setISRCallback(decaDevice_t device, decaISRCallback_t callback, void* context);

void deca_isr(decaDevice_t selectedDevice);

#ifdef __cplusplus
}
#endif

#endif // PORT_H
