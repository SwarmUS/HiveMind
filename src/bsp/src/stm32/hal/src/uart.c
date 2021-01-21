#include "hal/uart_host.h"
#include "hal/uart_print.h"

/** @brief Callback on the completion of the interrupt transfer */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == HUART_PRINT) {
        UartPrint_sendTxCallback();
    } else if (huart == HUART_HOST) {
        UartHost_txCallback();
    }
}

/** @brief Callback on the completion of the interrupt reception */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == HUART_HOST) {
        UartHost_rxCallback();
    }
}