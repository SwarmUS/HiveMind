#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hivemind_hal.h"
#include <deca_device_api.h>
#include <stdbool.h>

/**
 * @brief Enum to specify which decawave we are addressing
 */
typedef enum { DW_A = 0, DW_B = 1 } decaDevice_t;

typedef void (*decaISRCallback_t)(void* context);

/**
 * @brief Structure to hold a port and pin associated with a decawave NSS pin
 */
typedef struct {
    SPI_HandleTypeDef* spiHandle;
    GPIO_TypeDef* nssPort;
    uint16_t nssPin;
    GPIO_TypeDef* irqPort;
    uint16_t irqPin;
    GPIO_TypeDef* resetPort;
    uint16_t resetPin;
    void* isrContext;
    decaISRCallback_t isrCallback;
} decawaveDeviceConfig_t;

extern decawaveDeviceConfig_t g_decawaveConfigs[2];

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
 * @param selectedDevice The device to change SPI speed on
 */
void deca_setSlowRate(decaDevice_t selectedDevice);

/**
 * @brief Sets decawave SPI to fast rate (used once PLL has locked)
 * @param selectedDevice The device to change SPI speed on
 */
void deca_setFastRate(decaDevice_t selectedDevice);

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
decawaveDeviceConfig_t* deca_getSelectedDeviceConfig();

/**
 * @brief Gets the NSS configuration for a given decawave
 * @return The NSS pin
 */
decawaveDeviceConfig_t* deca_getDeviceConfig(decaDevice_t device);

/**
 * @brief Sets the ISR for a given decawave chip
 * @param device The decawave chip
 * @param callback The ISR callback
 * @param context A context to pass to the callback
 */
void deca_setISRCallback(decaDevice_t device, decaISRCallback_t callback, void* context);

/**
 * @brief The entry point for all decawave interrupts
 * @param selectedDevice The device that triggered the interrupt
 */
void deca_isr(decaDevice_t selectedDevice);

/**
 * @brief Sends a pulse on the DW clock sync
 */
void deca_pulseSyncSignal();

#ifdef __cplusplus
}
#endif

#endif // PORT_H
