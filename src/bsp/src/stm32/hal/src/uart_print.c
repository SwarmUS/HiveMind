/** @brief This file provides the callbacks and definition for using printf function */

#include "hal/uart_print.h"
#include <c-common/circular_buff.h>

#define CBUFF_HUART_PRINT_DATA_SIZE 2048

CircularBuff cbuffUartPrint;
uint16_t lastUartPrintTransferSize = 0;
static uint8_t cbuffUartPrintData[CBUFF_HUART_PRINT_DATA_SIZE];

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE* f)
#endif /* __GNUC__ */

/** Prototype definitions **/
PUTCHAR_PROTOTYPE {
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART3 and Loop until the end of transmission */
    volatile CircularBuffRet ret;
    if (HAL_UART_GetState(HUART_PRINT) == HAL_UART_STATE_READY) {
        HAL_UART_Transmit_IT(HUART_PRINT, (uint8_t*)&ch, 1);
    } else {
        ret = CircularBuff_putc(&cbuffUartPrint, ch);
        // Error handling
        if (ret != CircularBuff_Ret_Ok) {
            uint8_t buffErrMsg[] = "UART3 buffer full, clearing\r\n";
            lastUartPrintTransferSize = 0;
            HAL_UART_Abort(HUART_PRINT);
            HAL_UART_Transmit(HUART_PRINT, buffErrMsg, sizeof(buffErrMsg), HAL_MAX_DELAY);
            CircularBuff_clear(&cbuffUartPrint);
        }
    }
    return ch;
}

/** End prototype definitions **/

/** Function definitions **/

void UartPrint_init() {
    CircularBuff_init(&cbuffUartPrint, cbuffUartPrintData, CBUFF_HUART_PRINT_DATA_SIZE);
}

void UartPrint_sendTxCallback() {
    // Advance to the size of the last transfer
    CircularBuff_advance(&cbuffUartPrint, lastUartPrintTransferSize);

    // Send data
    ZeroCopyBuff buff = CircularBuff_getZeroCopy(&cbuffUartPrint, UINT16_MAX);
    if (buff.status == CircularBuff_Ret_Ok) {
        HAL_UART_Transmit_IT(HUART_PRINT, (uint8_t*)buff.data, buff.length);
    }

    lastUartPrintTransferSize = buff.length;
}

/** End Function definitions **/
