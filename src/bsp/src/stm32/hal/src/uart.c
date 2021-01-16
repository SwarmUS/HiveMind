#include "hal/uart_phone.h"
#include "hal/uart_print.h"

/** @brief Callback on the completion of the interrupt transfer */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == HUART_PRINT) {
        UartPrint_sendTxCallback();
    }
}

/** @brief Callback on the completion of the interrupt reception */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == HUART_PHONE) {
        UartPhone_rxCallback();
    }
}