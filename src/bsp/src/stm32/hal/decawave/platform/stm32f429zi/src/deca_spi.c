/*! ----------------------------------------------------------------------------
 * @file    deca_spi.c
 * @brief   SPI access functions
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include "deca_device_api.h"
#include "hivemind_hal.h"
#include "main.h"
#include "port.h"
#include "stm32f4xx_hal_def.h"

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success
 */
#pragma GCC optimize("O3")
int writetospi(uint16_t headerLength,
               const uint8_t* headerBuffer,
               uint32_t bodyLength,
               const uint8_t* bodyBuffer) {
    decaIrqStatus_t stat;
    stat = decamutexon();

    while (HAL_SPI_GetState(DW_SPI) != HAL_SPI_STATE_READY) {
    }

    HAL_GPIO_WritePin(DW_NSS_A_GPIO_Port, DW_NSS_A_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(DW_SPI, (uint8_t*)&headerBuffer[0], headerLength, HAL_MAX_DELAY);
    HAL_SPI_Transmit(DW_SPI, (uint8_t*)&bodyBuffer[0], bodyLength, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(DW_NSS_A_GPIO_Port, DW_NSS_A_Pin,
                      GPIO_PIN_SET); /**< Put chip select line high */

    decamutexoff(stat);

    return 0;
} // end writetospi()

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns 0
 */
#pragma GCC optimize("O3")
int readfromspi(uint16_t headerLength,
                const uint8_t* headerBuffer,
                uint32_t readlength,
                uint8_t* readBuffer) {
    int i;
    decaIrqStatus_t stat;
    stat = decamutexon();

    /* Blocking: Check whether previous transfer has been finished */
    while (HAL_SPI_GetState(DW_SPI) != HAL_SPI_STATE_READY)
        ;

    HAL_GPIO_WritePin(DW_NSS_A_GPIO_Port, DW_NSS_A_Pin,
                      GPIO_PIN_RESET); /**< Put chip select line low */

    /* Send header */
    for (i = 0; i < headerLength; i++) {
        HAL_SPI_Transmit(DW_SPI, (uint8_t*)&headerBuffer[i], 1, HAL_MAX_DELAY); // No timeout
    }

    /* for the data buffer use LL functions directly as the HAL SPI read function
     * has issue reading single bytes */
    while (readlength-- > 0) {
        /* Wait until TXE flag is set to send data */
        while (__HAL_SPI_GET_FLAG(DW_SPI, SPI_FLAG_TXE) == RESET) {
        }

        DW_SPI->Instance->DR = 0; /* set output to 0 (MOSI), this is necessary for
        e.g. when waking up DW1000 from DEEPSLEEP via dwt_spicswakeup() function.
        */

        /* Wait until RXNE flag is set to read data */
        while (__HAL_SPI_GET_FLAG(DW_SPI, SPI_FLAG_RXNE) == RESET) {
        }

        (*readBuffer++) = DW_SPI->Instance->DR; // copy data read form (MISO)
    }

    HAL_GPIO_WritePin(DW_NSS_A_GPIO_Port, DW_NSS_A_Pin,
                      GPIO_PIN_SET); /**< Put chip select line high */

    decamutexoff(stat);

    return 0;
} // end readfromspi()