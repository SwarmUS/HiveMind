#include "hal/esp_spi.h"

static spiCallbackFct_t rxCpltCallbackFct = NULL;
static spiCallbackFct_t txCpltCallbackFct = NULL;
static spiCallbackFct_t txrxCpltCallbackFct = NULL;
static void* rxCallbackContext = NULL;
static void* txCallbackContext = NULL;
static void* txrxCallbackContext = NULL;

bool EspSpi_TransmitDMA(const uint8_t* buffer,
                        uint16_t length,
                        spiCallbackFct_t cpltCallback,
                        void* context) {
    if (HAL_SPI_GetState(ESP_SPI) != HAL_SPI_STATE_BUSY_TX) {
        txCpltCallbackFct = cpltCallback;
        txCallbackContext = context;
        if (HAL_SPI_Transmit_DMA(ESP_SPI, (uint8_t*)buffer, length) == HAL_OK) {
            return true;
        }
    }
    return false;
}

bool EspSpi_ReceiveDma(uint8_t* buffer,
                       uint16_t length,
                       spiCallbackFct_t cpltCallback,
                       void* context) {
    if (HAL_SPI_GetState(ESP_SPI) != HAL_SPI_STATE_BUSY_RX) {
        txCpltCallbackFct = cpltCallback;
        txCallbackContext = context;
        if (HAL_SPI_Receive_DMA(ESP_SPI, buffer, length) == HAL_OK) {
            return true;
        }
    }
    return false;
}

bool EspSpi_TransmitReceiveDma(const uint8_t* txBuffer,
                               uint8_t* rxBuffer,
                               uint16_t lengthBytes,
                               spiCallbackFct_t cpltCallback,
                               void* context) {
    if (HAL_SPI_GetState(ESP_SPI) != HAL_SPI_STATE_BUSY_TX_RX) {
        txrxCpltCallbackFct = cpltCallback;
        txrxCallbackContext = context;

#ifdef SYSTEM_STM32H7XX_H
        // Clear cache regarding these addresses for transfers, especially on RX
        // See https://community.st.com/s/article/FAQ-DMA-is-not-working-on-STM32H7-devices,
        // solution 3
        SCB_CleanDCache_by_Addr((uint32_t*)(((uint32_t)txBuffer) & ~(uint32_t)0x1F),
                                lengthBytes + 32);
        SCB_CleanDCache_by_Addr((uint32_t*)(((uint32_t)rxBuffer) & ~(uint32_t)0x1F),
                                lengthBytes + 32);
        HAL_StatusTypeDef status =
            HAL_SPI_TransmitReceive_IT(ESP_SPI, (uint8_t*)txBuffer, rxBuffer, lengthBytes);
#else
        HAL_StatusTypeDef status =
            HAL_SPI_TransmitReceive_DMA(ESP_SPI, (uint8_t*)txBuffer, rxBuffer, lengthBytes);
#endif

        if (status == HAL_OK) {
            return true;
        }
    }
    return false;
}

void EspSpi_WriteCS(bool state) {
    HAL_GPIO_WritePin(ESP_CS_GPIO_Port, ESP_CS_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool EspSpi_ReadCS() { return HAL_GPIO_ReadPin(ESP_CS_GPIO_Port, ESP_CS_Pin) == GPIO_PIN_SET; }

void EspSpi_RxCallback() {
    if (rxCpltCallbackFct != NULL && rxCallbackContext != NULL) {
        rxCpltCallbackFct(rxCallbackContext);
    }
}

void EspSpi_TxCallback() {
    if (txCpltCallbackFct != NULL && txCallbackContext != NULL) {
        txCpltCallbackFct(txCallbackContext);
    }
}

void EspSpi_TxRxCallback() {
    if (txrxCpltCallbackFct != NULL && txrxCallbackContext != NULL) {
        txrxCpltCallbackFct(txrxCallbackContext);
    }
}
