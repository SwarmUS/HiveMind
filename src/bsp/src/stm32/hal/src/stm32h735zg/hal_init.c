#include "hal/hal_init.h"
#include <hivemind_hal.h>
#include <tim.h>

void Hal_initPlatformSpecific() {
    // UART Print
    MX_USART3_UART_Init();

    // Decawave SPI
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_SPI3_Init();

    // ESP SPI
    MX_SPI5_Init();

    // Heartbeat timer
    MX_TIM6_Init();

    // IO Expander
    MX_I2C1_Init();
}
