#include "hal/hal_init.h"
#include <hivemind_hal.h>
#include <tim.h>

static void Hal_initMPU();
static void Hal_initCache();

void PHal_initMcu() {
    // UART Print
    MX_USART3_UART_Init();

    // Decawave SPI
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_SPI3_Init();

    // ESP SPI
    MX_SPI5_Init();

    // Heartbeat timer
    MX_TIM13_Init();

    // IO Expander
    MX_I2C1_Init();

    // TODO: Reactivate once USB voltage input is fixed
    // MX_USB_DEVICE_Init();
    // usb_init();

    Hal_initMPU();
    Hal_initCache();

    HAL_Delay(500);
}

static void Hal_initMPU() {
    // Taken from the STM32H735G-DK lwIP example
    MPU_Region_InitTypeDef MPU_InitStruct;

    /* Disable the MPU */
    HAL_MPU_Disable();

    /* Configure the MPU attributes as Device not cacheable
       for ETH DMA descriptors */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x30000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* Configure the MPU attributes as Normal Non Cacheable
       for LwIP RAM heap which contains the Tx buffers */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x30004000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER1;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* Enable the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

static void Hal_initCache() {
    /* Enable I-Cache */
    SCB_EnableICache();

    /* Enable D-Cache */
    SCB_EnableDCache();
}

bool Hal_wroomPowerEnabled() {
    return HAL_GPIO_ReadPin(WROOM_PWR_EN_GPIO_Port, WROOM_PWR_EN_Pin) == GPIO_PIN_SET;
}
void Hal_enableWroom() { HAL_GPIO_WritePin(WROOM_EN_GPIO_Port, WROOM_EN_Pin, GPIO_PIN_SET); }

bool Hal_ethernetPowerEnabled() {
    return HAL_GPIO_ReadPin(ETH_PWR_EN_GPIO_Port, ETH_PWR_EN_Pin) == GPIO_PIN_SET;
}
void Hal_enableEthernet() { HAL_GPIO_WritePin(PHY_nRST_GPIO_Port, PHY_nRST_Pin, GPIO_PIN_SET); }