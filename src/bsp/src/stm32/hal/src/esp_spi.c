#include "hal/esp_spi.h"

static spiCallbackFct rxCpltCallbackFct = NULL;
static spiCallbackFct txCpltCallbackFct = NULL;
static spiCallbackFct txrxCpltCallbackFct = NULL;
static void* rxCallbackInstance = NULL;
static void* txCallbackInstance = NULL;
static void* txrxCallbackInstance = NULL;

bool EspSpi_TransmitDMA(const uint8_t* buffer,
                        uint16_t length,
                        spiCallbackFct cpltCallback,
                        void* instance) {
    bool success = false;
    if (HAL_SPI_GetState(ESP_SPI) != HAL_SPI_STATE_BUSY_TX) {
        txCpltCallbackFct = cpltCallback;
        txCallbackInstance = instance;
        if (HAL_SPI_Transmit_DMA(ESP_SPI, buffer, length) == HAL_OK) {
            success = true;
        }
    }
    return success;
}

bool EspSpi_ReceiveDma(const uint8_t* buffer,
                       uint16_t length,
                       spiCallbackFct cpltCallback,
                       void* instance) {
    bool success = false;
    if (HAL_SPI_GetState(ESP_SPI) != HAL_SPI_STATE_BUSY_RX) {
        txCpltCallbackFct = cpltCallback;
        txCallbackInstance = instance;
        if (HAL_SPI_Receive_DMA(ESP_SPI, buffer, length) == HAL_OK) {
            success = true;
        }
    }
    return success;
}

bool EspSpi_TransmitReceiveDma(const uint8_t* txBuffer,
                               const uint8_t* rxBuffer,
                               uint16_t lengthBytes,
                               spiCallbackFct cpltCallback,
                               void* instance) {
    bool success = false;
    if (HAL_SPI_GetState(ESP_SPI) != HAL_SPI_STATE_BUSY_TX_RX) {
        txrxCpltCallbackFct = cpltCallback;
        txrxCallbackInstance = instance;
        HAL_StatusTypeDef status =
            HAL_SPI_TransmitReceive_DMA(ESP_SPI, txBuffer, rxBuffer, lengthBytes);
        if (status == HAL_OK) {
            success = true;
        }
    }
    return success;
}

void EspSpi_RxCallback() {
    if (rxCpltCallbackFct != NULL && rxCallbackInstance != NULL) {
        rxCpltCallbackFct(rxCallbackInstance);
    }
}

void EspSpi_TxCallback() {
    if (txCpltCallbackFct != NULL && txCallbackInstance != NULL) {
        txCpltCallbackFct(txCallbackInstance);
    }
}

void EspSpi_TxRxCallback() {
    if (txrxCpltCallbackFct != NULL && txrxCallbackInstance != NULL) {
        txrxCpltCallbackFct(txrxCallbackInstance);
    }
}