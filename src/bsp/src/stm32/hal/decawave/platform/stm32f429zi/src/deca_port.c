#include "deca_port.h"
#include "main.h"
#include "stm32f4xx_hal_conf.h"

extern dwt_local_data_t dw1000local[DWT_NUM_DW_DEV];
extern dwt_local_data_t* pdw1000local;

static decaNSSConfig_t g_decaNssConfigA = {DW_NSS_A_GPIO_Port, DW_NSS_A_Pin, &dw1000local[0]};
static decaNSSConfig_t g_decaNssConfigB = {DW_NSS_B_GPIO_Port, DW_NSS_B_Pin, &dw1000local[1]};

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
    DW_SPI->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    HAL_SPI_Init(DW_SPI);
}

void deca_init() {
    deca_hardwareReset(DW_A);
    deca_hardwareReset(DW_B);
    deca_setSlowRate();
}

void deca_selectDevice(decaDevice_t selectedDevice) {
    g_selectedDecaDevice = selectedDevice;
    pdw1000local = deca_getSelectedNSSConfig()->dwtLocalData;
}

decaNSSConfig_t* deca_getSelectedNSSConfig() {
    switch (g_selectedDecaDevice) {
    case DW_A:
        return &g_decaNssConfigA;

    case DW_B:
        return &g_decaNssConfigB;

    default:
        return NULL;
    }
}
