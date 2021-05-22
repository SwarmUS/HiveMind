#include "deca_port.h"
#include "deca_device_api.h"
#include "main.h"
#include <FreeRTOS.h>
#include <task.h>

static decaDevice_t g_selectedDecaDevice = 0;

void deca_hardwareReset(decaDevice_t selectedDevice) {
    decawaveDeviceConfig_t* decaConfig = deca_getDeviceConfig(selectedDevice);
    GPIO_InitTypeDef gpioInitStruct;

    // Enable GPIO used for DW1000 reset as open collector output
    gpioInitStruct.Pin = decaConfig->resetPin;
    gpioInitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    gpioInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(decaConfig->resetPort, &gpioInitStruct);

    // drive the RSTn pin low
    HAL_GPIO_WritePin(decaConfig->resetPort, decaConfig->resetPin, GPIO_PIN_RESET);

    // Use HAL functions instead of FreeRTOS delays as this is called before the scheduler is
    // started
    HAL_Delay(1);

    // put the pin back to tri-state ... as
    // output open-drain (not active)
    gpioInitStruct.Pin = decaConfig->resetPin;
    gpioInitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    gpioInitStruct.Pull = GPIO_NOPULL;
    gpioInitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(decaConfig->resetPort, &gpioInitStruct);
    HAL_GPIO_WritePin(decaConfig->resetPort, decaConfig->resetPin, GPIO_PIN_SET);

    HAL_Delay(100);
}

void deca_setSlowRate(decaDevice_t selectedDevice) {
    decawaveDeviceConfig_t* decaConfig = deca_getDeviceConfig(selectedDevice);

    decaConfig->spiHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    HAL_SPI_Init(decaConfig->spiHandle);
}

void deca_setFastRate(decaDevice_t selectedDevice) {
    decawaveDeviceConfig_t* decaConfig = deca_getDeviceConfig(selectedDevice);

    decaConfig->spiHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    HAL_SPI_Init(decaConfig->spiHandle);
}

void deca_init() {
    for (int i = 0; i < DWT_NUM_DW_DEV; i++) {
        deca_hardwareReset(i);
        deca_setSlowRate(i);
    }
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
    if (deviceConfig == NULL) {
        return;
    }

    while (HAL_GPIO_ReadPin(deviceConfig->irqPort, deviceConfig->irqPin) != GPIO_PIN_RESET) {
        if (deviceConfig->isrCallback) {
            deviceConfig->isrCallback(deviceConfig->isrContext);
        }
    }
}
