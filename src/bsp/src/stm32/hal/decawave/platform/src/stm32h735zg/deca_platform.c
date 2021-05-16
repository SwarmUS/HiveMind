#include "deca_port.h"
#include <FreeRTOS.h>
#include <task.h>

void deca_setSync(bool state) {
    HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void deca_pulseSyncSignal() {
    deca_setSync(true);
    vTaskDelay(1);
    deca_setSync(false);
}

decawaveDeviceConfig_t g_decawaveConfigs[DWT_NUM_DW_DEV] = {{.spiHandle = &hspi1, // A0
                                                             .nssPort = SPI_nCS_0A_GPIO_Port,
                                                             .nssPin = SPI_nCS_0A_Pin,
                                                             .irqPort = CHAN_IRQ_0A_GPIO_Port,
                                                             .irqPin = CHAN_IRQ_0A_Pin,
                                                             .resetPort = CHAN_RESET_0A_GPIO_Port,
                                                             .resetPin = CHAN_RESET_0A_Pin,
                                                             .isrCallback = NULL,
                                                             .isrContext = NULL},
                                                            {.spiHandle = &hspi1, // A1
                                                             .nssPort = SPI_nCS_1A_GPIO_Port,
                                                             .nssPin = SPI_nCS_1A_Pin,
                                                             .irqPort = CHAN_IRQ_1A_GPIO_Port,
                                                             .irqPin = CHAN_IRQ_1A_Pin,
                                                             .resetPort = CHAN_RESET_1A_GPIO_Port,
                                                             .resetPin = CHAN_RESET_1A_Pin,
                                                             .isrCallback = NULL,
                                                             .isrContext = NULL},
                                                            {.spiHandle = &hspi3, // B0
                                                             .nssPort = SPI_nCS_0B_GPIO_Port,
                                                             .nssPin = SPI_nCS_0B_Pin,
                                                             .irqPort = CHAN_IRQ_0B_GPIO_Port,
                                                             .irqPin = CHAN_IRQ_0B_Pin,
                                                             .resetPort = CHAN_RESET_0B_GPIO_Port,
                                                             .resetPin = CHAN_RESET_0B_Pin,
                                                             .isrCallback = NULL,
                                                             .isrContext = NULL},
                                                            {.spiHandle = &hspi3, // B1
                                                             .nssPort = SPI_nCS_1B_GPIO_Port,
                                                             .nssPin = SPI_nCS_1B_Pin,
                                                             .irqPort = CHAN_IRQ_1B_GPIO_Port,
                                                             .irqPin = CHAN_IRQ_1B_Pin,
                                                             .resetPort = CHAN_RESET_1B_GPIO_Port,
                                                             .resetPin = CHAN_RESET_1B_Pin,
                                                             .isrCallback = NULL,
                                                             .isrContext = NULL},
                                                            {.spiHandle = &hspi2, // C0
                                                             .nssPort = SPI_nCS_0C_GPIO_Port,
                                                             .nssPin = SPI_nCS_0C_Pin,
                                                             .irqPort = CHAN_IRQ_0C_GPIO_Port,
                                                             .irqPin = CHAN_IRQ_0C_Pin,
                                                             .resetPort = CHAN_RESET_0C_GPIO_Port,
                                                             .resetPin = CHAN_RESET_0C_Pin,
                                                             .isrCallback = NULL,
                                                             .isrContext = NULL},
                                                            {.spiHandle = &hspi2, // C1
                                                             .nssPort = SPI_nCS_1C_GPIO_Port,
                                                             .nssPin = SPI_nCS_1C_Pin,
                                                             .irqPort = CHAN_IRQ_1C_GPIO_Port,
                                                             .irqPin = CHAN_IRQ_1C_Pin,
                                                             .resetPort = CHAN_RESET_1C_GPIO_Port,
                                                             .resetPin = CHAN_RESET_1C_Pin,
                                                             .isrCallback = NULL,
                                                             .isrContext = NULL}};