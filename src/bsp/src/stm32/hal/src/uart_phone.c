#include "hal/uart_phone.h"

void (*rxCpltCallbackFct)() = NULL;

bool UartPhone_transmitBuffer(const uint8_t* buffer, uint16_t length) {
    bool ret = true;

    if (HAL_UART_GetState(HUART_PRINT) == HAL_UART_STATE_READY) {
        HAL_UART_Transmit_DMA(HUART_PHONE, buffer, length);
    } else {
        ret = false;
    }

    return false;
}

bool UartPhone_receiveDMA(const uint8_t* buffer, uint16_t length, void (*cpltCallback)()) {
    bool ret = true;

    if (HAL_UART_GetState(HUART_PRINT) == HAL_UART_STATE_READY) {
        rxCpltCallbackFct = cpltCallback;
        HAL_UART_Receive_DMA(HUART_PHONE, buffer, length);
    } else {
        ret = false;
    }

    return ret;
}

void UartPhone_rxCallback() {
    if (rxCpltCallbackFct != NULL) {
        rxCpltCallbackFct();
    }
}