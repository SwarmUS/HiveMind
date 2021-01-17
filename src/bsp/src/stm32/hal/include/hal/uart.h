#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

void UART_IRQHandler(UART_HandleTypeDef* huart);

#ifdef __cplusplus
}
#endif

#endif