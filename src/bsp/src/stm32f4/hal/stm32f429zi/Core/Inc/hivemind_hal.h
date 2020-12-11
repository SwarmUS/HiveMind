#include "gpio.h"
#include "main.h"
#include "stm32f4xx_hal_conf.h"
#include "usart.h"

UART_HandleTypeDef* huart_print = &huart3;