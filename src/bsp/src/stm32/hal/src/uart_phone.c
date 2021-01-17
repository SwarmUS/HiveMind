#include "hal/uart_phone.h"

volatile void (*rxCpltCallbackFct)(void*) = NULL;
volatile void* rxCallbackInstance = NULL;
volatile void (*txCpltCallbackFct)(void*) = NULL;
volatile void* txCallbackInstance = NULL;

bool UartPhone_transmitBuffer(const uint8_t* buffer,
                              uint16_t length,
                              void (*cpltCallback)(void*),
                              void* instance) {
    bool ret = true;

    if (HAL_UART_GetState(HUART_PHONE) != HAL_UART_STATE_BUSY_TX) {
        txCpltCallbackFct = cpltCallback;
        txCallbackInstance = instance;
        if (HAL_UART_Transmit_DMA(HUART_PHONE, buffer, length) != HAL_OK) {
            ret = false;
        }
    } else {
        ret = false;
    }

    return ret;
}

bool UartPhone_receiveDMA(const uint8_t* buffer,
                          uint16_t length,
                          void (*cpltCallback)(void*),
                          void* instance) {
    bool ret = true;

    if (HAL_UART_GetState(HUART_PHONE) != HAL_UART_STATE_BUSY_RX) {
        rxCpltCallbackFct = cpltCallback;
        rxCallbackInstance = instance;
        HAL_UART_Receive_DMA(HUART_PHONE, buffer, length);
    } else {
        ret = false;
    }

    return ret;
}

void UartPhone_rxCallback() {
    if (rxCpltCallbackFct != NULL && rxCallbackInstance != NULL) {
        rxCpltCallbackFct(rxCallbackInstance);
    }
}

void UartPhone_txCallback() {
    if (txCpltCallbackFct != NULL && txCallbackInstance != NULL) {
        txCpltCallbackFct(txCallbackInstance);
    }
}