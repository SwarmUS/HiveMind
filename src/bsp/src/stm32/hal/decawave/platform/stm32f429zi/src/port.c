#include <sys/types.h>

#include "deca_device_api.h"
#include "main.h"
#include "port.h"
#include "stm32f4xx_hal_conf.h"

/**
 * @brief Precise microsecond sleep for stm32f429zi
 * @param usec number of microseconds to sleep for
 */
#pragma GCC optimize("O0")
void usleep(useconds_t usec) {
    int i, j;
#pragma GCC ivdep
    for (i = 0; i < usec; i++) {
#pragma GCC ivdep
        for (j = 0; j < 2; j++) {
            __NOP();
            __NOP();
        }
    }
}

void decawave_reset() {
    GPIO_InitTypeDef GPIO_InitStruct;

    // Enable GPIO used for DW1000 reset as open collector output
    GPIO_InitStruct.Pin = DW_A_NRST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DW_A_NRST_GPIO_Port, &GPIO_InitStruct);

    // drive the RSTn pin low
    HAL_GPIO_WritePin(DW_A_NRST_GPIO_Port, DW_A_NRST_Pin, GPIO_PIN_RESET);

    usleep(1);

    // put the pin back to tri-state ... as
    // output open-drain (not active)
    GPIO_InitStruct.Pin = DW_A_NRST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DW_A_NRST_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(DW_A_NRST_GPIO_Port, DW_A_NRST_Pin, GPIO_PIN_SET);

    deca_sleep(2);
}

void decawave_wakeup() {
    HAL_GPIO_WritePin(DW_A_NSS_GPIO_Port, DW_A_NSS_Pin, GPIO_PIN_RESET);
    deca_sleep(1);
    HAL_GPIO_WritePin(DW_A_NSS_GPIO_Port, DW_A_NSS_Pin, GPIO_PIN_SET);
    deca_sleep(7); // wait 7ms for DW1000 XTAL to stabilise
}

void decawave_setSlowRate() {
    DW_SPI->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    HAL_SPI_Init(DW_SPI);
}

void decawave_setFastRate() {
    DW_SPI->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    HAL_SPI_Init(DW_SPI);
}