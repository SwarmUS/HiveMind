#include "deca_port.h"
#include "main.h"
#include "stm32f4xx_hal_conf.h"

void deca_hardwareReset() {
    GPIO_InitTypeDef GPIO_InitStruct;

    // Enable GPIO used for DW1000 reset as open collector output
    GPIO_InitStruct.Pin = DW_RESET_A_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DW_RESET_A_GPIO_Port, &GPIO_InitStruct);

    // drive the RSTn pin low
    HAL_GPIO_WritePin(DW_RESET_A_GPIO_Port, DW_RESET_A_Pin, GPIO_PIN_RESET);

    // Use HAL functions instead of FreeRTOS delays as this is called before the scheduler is
    // started
    HAL_Delay(1);

    // put the pin back to tri-state ... as
    // output open-drain (not active)
    GPIO_InitStruct.Pin = DW_RESET_A_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DW_RESET_A_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(DW_RESET_A_GPIO_Port, DW_RESET_A_Pin, GPIO_PIN_SET);

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
    deca_hardwareReset();
    deca_setSlowRate();
}