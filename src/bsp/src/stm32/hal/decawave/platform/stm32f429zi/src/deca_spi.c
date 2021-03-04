#include "deca_device_api.h"
#include "hivemind_hal.h"
#include "main.h"
#include "stm32f4xx_hal_def.h"

/**
 * @brief Low-level SPI function used by Decawave driver to write bytes to the device
 * @param headerLength Length of header in bytes
 * @param headerBuffer Buffer of the header to send
 * @param bodyLength Length of body in bytes
 * @param bodyBuffer Buffer of the body to send
 * @return unused (only here for compatibility with deca driver)
 */
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
}

/**
 * @brief Low-level SPI function used by Decawave driver to read bytes from the device
 * @param headerLength Length of header in bytes
 * @param headerBuffer Buffer of the header to send
 * @param bodyLength Length of body in bytes
 * @param bodyBuffer Buffer of the body to receive
 * @return unused (only here for compatibility with deca driver)
 */
int readfromspi(uint16_t headerLength,
                const uint8_t* headerBuffer,
                uint32_t readlength,
                uint8_t* readBuffer) {
    int i;
    decaIrqStatus_t stat;
    stat = decamutexon();

    /* Blocking: Check whether previous transfer has been finished */
    while (HAL_SPI_GetState(DW_SPI) != HAL_SPI_STATE_READY) {
    }

    HAL_GPIO_WritePin(DW_NSS_A_GPIO_Port, DW_NSS_A_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(DW_SPI, headerBuffer, headerLength, HAL_MAX_DELAY);

    HAL_SPI_Receive(DW_SPI, readBuffer, readlength, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(DW_NSS_A_GPIO_Port, DW_NSS_A_Pin, GPIO_PIN_SET);

    decamutexoff(stat);

    return 0;
} // end readfromspi()