#include "stm32f4xx_hal_conf.h"
#include "main.h"
#include "usart.h"
#include "gpio.h"


UART_HandleTypeDef* huart_print = &huart3;