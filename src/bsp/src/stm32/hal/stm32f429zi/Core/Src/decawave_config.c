#include "hivemind_hal.h"

// TODO: Get num devices
decawaveDeviceConfig_t g_decawaveConfigs[2] = {{.spiHandle = &hspi4,
                                                .nssPort = DW_NSS_A_GPIO_Port,
                                                .nssPin = DW_NSS_A_Pin,
                                                .irqPort = DW_IRQn_A_GPIO_Port,
                                                .irqPin = DW_IRQn_A_Pin,
                                                .resetPort = DW_RESET_A_GPIO_Port,
                                                .resetPin = DW_RESET_A_Pin,
                                                .isrCallback = NULL,
                                                .isrContext = NULL},
                                               {.spiHandle = &hspi4,
                                                .nssPort = DW_NSS_B_GPIO_Port,
                                                .nssPin = DW_NSS_B_Pin,
                                                .irqPort = DW_IRQn_B_GPIO_Port,
                                                .irqPin = DW_IRQn_B_Pin,
                                                .resetPort = DW_RESET_B_GPIO_Port,
                                                .resetPin = DW_RESET_B_Pin,
                                                .isrCallback = NULL,
                                                .isrContext = NULL}};