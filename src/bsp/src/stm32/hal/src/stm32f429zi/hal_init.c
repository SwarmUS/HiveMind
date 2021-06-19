#include "hal/hal_init.h"
#include <hivemind_hal.h>

void Hal_initPlatformSpecific() {
    // UART Print
    MX_USART3_UART_Init();

    // Decawave SPI
    MX_SPI3_Init();

    // ESP SPI
    MX_SPI5_Init();
    MX_SPI4_Init();

    // Heartbeat timer
    MX_TIM6_Init();
}
