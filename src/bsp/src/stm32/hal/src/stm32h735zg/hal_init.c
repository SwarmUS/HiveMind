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

bool Hal_wroomPowerEnabled() {
    return HAL_GPIO_ReadPin(WROOM_PWR_EN_GPIO_Port, WROOM_PWR_EN_Pin) == GPIO_PIN_SET;
}
void Hal_enableWroom() { HAL_GPIO_WritePin(WROOM_EN_GPIO_Port, WROOM_EN_Pin, GPIO_PIN_SET); }

bool Hal_ethernetPowerEnabled() {
    return HAL_GPIO_ReadPin(ETH_PWR_EN_GPIO_Port, ETH_PWR_EN_Pin) == GPIO_PIN_SET;
}
void Hal_enableEthernet() { HAL_GPIO_WritePin(PHY_nRST_GPIO_Port, PHY_nRST_Pin, GPIO_PIN_SET); }
