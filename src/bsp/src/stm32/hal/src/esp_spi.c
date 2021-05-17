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
        HAL_StatusTypeDef status =
            HAL_SPI_TransmitReceive_DMA(ESP_SPI, (uint8_t*)txBuffer, rxBuffer, lengthBytes);
        if (status == HAL_OK) {
            return true;
        }
    }
    return false;
}

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