#include "hal/hal_init.h"
#include <hal/usb.h>
#include <hivemind_hal.h>
#include <usb_device.h>

void PHal_initMcu() {
    // UART Print
    MX_USART3_UART_Init();

    // Decawave SPI
    MX_SPI3_Init();

    // ESP SPI
    MX_SPI5_Init();
    MX_SPI4_Init();

    // Heartbeat timer
    MX_TIM6_Init();

    MX_USB_DEVICE_Init();
    usb_init();
}

bool Hal_wroomPowerEnabled() { return true; }
void Hal_enableWroom() { HAL_GPIO_WritePin(WROOM_EN_GPIO_Port, WROOM_EN_Pin, GPIO_PIN_SET); }

bool Hal_ethernetPowerEnabled() { return true; }
void Hal_enableEthernet() {
    // Nothing to do here on the F4
}
