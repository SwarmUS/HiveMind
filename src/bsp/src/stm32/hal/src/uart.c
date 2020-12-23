#include "hal/uart_print.h"

/** @brief Callback on the completion of the interrupt transfer */ 
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == HUART_PRINT) {
        UartPrint_sendTxCallback();
    }
}