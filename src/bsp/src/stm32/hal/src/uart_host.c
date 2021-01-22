#include "hal/uart_host.h"

static uartCallbackFct rxCpltCallbackFct = NULL;
static uartCallbackFct txCpltCallbackFct = NULL;
static void* rxCallbackInstance = NULL;
static void* txCallbackInstance = NULL;

bool UartHost_transmitBuffer(const uint8_t* buffer,
                             uint16_t length,
                             uartCallbackFct cpltCallback,
                             void* instance) {
    bool ret = true;

    if (HAL_UART_GetState(HUART_HOST) != HAL_UART_STATE_BUSY_TX) {
        txCpltCallbackFct = cpltCallback;
        txCallbackInstance = instance;
        if (HAL_UART_Transmit_DMA(HUART_HOST, buffer, length) != HAL_OK) {
            ret = false;
        }
    } else {
        ret = false;
    }

    return ret;
}

bool UartHost_receiveDMA(const uint8_t* buffer,
                         uint16_t length,
                         uartCallbackFct cpltCallback,
                         void* instance) {
    bool ret = true;

    if (HAL_UART_GetState(HUART_HOST) != HAL_UART_STATE_BUSY_RX) {
        rxCpltCallbackFct = cpltCallback;
        rxCallbackInstance = instance;
        HAL_UART_Receive_DMA(HUART_HOST, buffer, length);
    } else {
        ret = false;
    }

    return ret;
}

void UartHost_rxCallback() {
    if (rxCpltCallbackFct != NULL && rxCallbackInstance != NULL) {
        rxCpltCallbackFct(rxCallbackInstance);
    }
}

void UartHost_txCallback() {
    if (txCpltCallbackFct != NULL && txCallbackInstance != NULL) {
        txCpltCallbackFct(txCallbackInstance);
    }
}