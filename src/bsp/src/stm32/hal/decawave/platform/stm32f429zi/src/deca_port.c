#include "deca_port.h"
#include "deca_device_api.h"
#include "main.h"
#include "stm32f4xx_hal_conf.h"

static decawaveDeviceConfig_t g_decawaveConfigs[DWT_NUM_DW_DEV] = {
    {DW_NSS_A_GPIO_Port, DW_NSS_A_Pin, DW_IRQn_A_GPIO_Port, DW_IRQn_A_Pin, NULL, NULL},
    {DW_NSS_B_GPIO_Port, DW_NSS_B_Pin, DW_IRQn_B_GPIO_Port, DW_IRQn_B_Pin, NULL, NULL}};

static decaDevice_t g_selectedDecaDevice = DW_A;

void deca_hardwareReset(decaDevice_t selectedDevice) {
    GPIO_TypeDef* selectedDecaResetPort;
    uint16_t selectedDecaResetPin;

    switch (selectedDevice) {
    case DW_A:
        selectedDecaResetPin = DW_RESET_A_Pin;
        selectedDecaResetPort = DW_RESET_A_GPIO_Port;
        break;

    case DW_B:
        selectedDecaResetPin = DW_RESET_B_Pin;
        selectedDecaResetPort = DW_RESET_B_GPIO_Port;
        break;

    default:
        return;
    }

    GPIO_InitTypeDef gpioInitStruct;

    // Enable GPIO used for DW1000 reset as open collector output
    gpioInitStruct.Pin = selectedDecaResetPin;
    gpioInitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    gpioInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(selectedDecaResetPort, &gpioInitStruct);

    // drive the RSTn pin low
    HAL_GPIO_WritePin(selectedDecaResetPort, selectedDecaResetPin, GPIO_PIN_RESET);

    // Use HAL functions instead of FreeRTOS delays as this is called before the scheduler is
    // started
    HAL_Delay(1);

    // put the pin back to tri-state ... as
    // output open-drain (not active)
    gpioInitStruct.Pin = selectedDecaResetPin;
    gpioInitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    gpioInitStruct.Pull = GPIO_NOPULL;
    gpioInitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(selectedDecaResetPort, &gpioInitStruct);
    HAL_GPIO_WritePin(selectedDecaResetPort, selectedDecaResetPin, GPIO_PIN_SET);

    HAL_Delay(100);
}

void deca_hardwareWakeup() {
    HAL_GPIO_WritePin(DW_RESET_A_GPIO_Port, DW_RESET_A_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(DW_RESET_A_GPIO_Port, DW_RESET_A_Pin, GPIO_PIN_SET);
    HAL_Delay(7); // wait 7ms for DW1000 XTAL to stabilise
}

void deca_setSlowRate() {
    DW_SPI->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    HAL_SPI_Init(DW_SPI);
}

void deca_setFastRate() {
    DW_SPI->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    HAL_SPI_Init(DW_SPI);
}

void deca_init() {
    deca_hardwareReset(DW_A);
    deca_hardwareReset(DW_B);
    deca_setSlowRate();
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

void deca_setSync(bool state) {
    HAL_GPIO_WritePin(DW_SYNC_GPIO_Port, DW_SYNC_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void deca_setSyncEnable(bool state) {
    HAL_GPIO_WritePin(DW_SYNC_GPIO_Port, DW_SYNC_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void deca_setSyncClear(bool state) {
    HAL_GPIO_WritePin(DW_SYNC_GPIO_Port, DW_SYNC_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
