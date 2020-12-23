#ifndef __UART_PRINT_H__
#define __UART_PRINT_H__

#include "common/circular_buff.h"
#include "hivemind_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize the uart print, after this call, you can use printf as usual
 */
void UartPrint_init();

#ifdef __cplusplus
}
#endif

#endif // __UART_PRINT_H__