#include "deca_port.h"
#include "deca_device_api.h"
#include "main.h"
#include <FreeRTOS.h>
#include <task.h>

static decaDevice_t g_selectedDecaDevice = 0;

void deca_setSlowRate(decaDevice_t selectedDevice) {
    decawaveDeviceConfig_t* decaConfig = deca_getDeviceConfig(selectedDevice);

    decaConfig->spiHandle->Init.BaudRatePrescaler = DECA_SPI_SLOW_RATE;
    HAL_SPI_Init(decaConfig->spiHandle);
}

void deca_setFastRate(decaDevice_t selectedDevice) {
    decawaveDeviceConfig_t* decaConfig = deca_getDeviceConfig(selectedDevice);

    decaConfig->spiHandle->Init.BaudRatePrescaler = DECA_SPI_FAST_RATE;
    HAL_SPI_Init(decaConfig->spiHandle);
}

void deca_selectDevice(decaDevice_t selectedDevice) {
    if (selectedDevice < DWT_NUM_DW_DEV) {
        g_selectedDecaDevice = selectedDevice;
        dwt_setSelectedDevice(selectedDevice);
    }
}

decawaveDeviceConfig_t* deca_getSelectedDeviceConfig() {
    return deca_getDeviceConfig(g_selectedDecaDevice);
}

decawaveDeviceConfig_t* deca_getDeviceConfig(decaDevice_t device) {
    if (device < DWT_NUM_DW_DEV) {
        return &g_decawaveConfigs[device];
    }

    return NULL;
}

void deca_setISRCallback(decaDevice_t device, decaISRCallback_t callback, void* context) {
    decawaveDeviceConfig_t* deviceConfig = deca_getDeviceConfig(device);

    if (deviceConfig != NULL) {
        deviceConfig->isrCallback = callback;
        deviceConfig->isrContext = context;
    }
}

void deca_isr(decaDevice_t selectedDevice) {
    decawaveDeviceConfig_t* deviceConfig = deca_getDeviceConfig(selectedDevice);
    if (deviceConfig == NULL || deca_getDeviceConfig(selectedDevice)->isPresent == false) {
        return;
    }

    while (HAL_GPIO_ReadPin(deviceConfig->irqPort, deviceConfig->irqPin) != DECA_IRQ_IDLE_STATE) {
        if (deviceConfig->isrCallback) {
            deviceConfig->isrCallback(deviceConfig->isrContext);
        }
    }
}

bool deca_isPresent(decaDevice_t selectedDevice) {
    decawaveDeviceConfig_t* deviceConfig = deca_getDeviceConfig(selectedDevice);
    if (deviceConfig == NULL) {
        return false;
    }

    return deviceConfig->isPresent;
}
