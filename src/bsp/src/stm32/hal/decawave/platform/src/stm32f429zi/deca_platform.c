#include "deca_port.h"
#include <FreeRTOS.h>
#include <task.h>

void deca_setSync(bool state) {
    HAL_GPIO_WritePin(DW_SYNC_GPIO_Port, DW_SYNC_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void deca_setSyncEnable(bool state) {
    HAL_GPIO_WritePin(DW_SYNC_EN_GPIO_Port, DW_SYNC_EN_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void deca_setSyncClear(bool state) {
    HAL_GPIO_WritePin(DW_SYNC_CLEAR_GPIO_Port, DW_SYNC_CLEAR_Pin,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void deca_pulseSyncSignal() {
    // Enable sync
    deca_setSyncEnable(true);
    deca_setSyncClear(true);

    // Sync
    // TODO: Maybe play on timings here
    deca_setSync(true);
    vTaskDelay(1);
    deca_setSync(false);
    deca_setSyncClear(false);

    // Disable sync
    deca_setSyncEnable(false);
}

void deca_init() {
    for (int i = 0; i < DWT_NUM_DW_DEV; i++) {
        deca_hardwareReset((decaDevice_t)i);
        deca_setSlowRate((decaDevice_t)i);
    }
}

decawaveDeviceConfig_t g_decawaveConfigs[DWT_NUM_DW_DEV] = {{.spiHandle = &hspi4,
                                                             .nssPort = DW_NSS_A_GPIO_Port,
                                                             .nssPin = DW_NSS_A_Pin,
                                                             .irqPort = DW_IRQn_A_GPIO_Port,
                                                             .irqPin = DW_IRQn_A_Pin,
                                                             .resetPort = DW_RESET_A_GPIO_Port,
                                                             .resetPin = DW_RESET_A_Pin,
                                                             .isrCallback = NULL,
                                                             .isrContext = NULL,
                                                             .isPresent = true},
                                                            {.spiHandle = &hspi4,
                                                             .nssPort = DW_NSS_B_GPIO_Port,
                                                             .nssPin = DW_NSS_B_Pin,
                                                             .irqPort = DW_IRQn_B_GPIO_Port,
                                                             .irqPin = DW_IRQn_B_Pin,
                                                             .resetPort = DW_RESET_B_GPIO_Port,
                                                             .resetPin = DW_RESET_B_Pin,
                                                             .isrCallback = NULL,
                                                             .isrContext = NULL,
                                                             .isPresent = true}};
