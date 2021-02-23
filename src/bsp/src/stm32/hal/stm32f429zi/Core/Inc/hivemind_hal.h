#ifndef __HIVEMIND_HAL_H__
#define __HIVEMIND_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "crc.h"
#include "dma.h"
#include "gpio.h"
#include "main.h"
#include "spi.h"
#include "stm32f4xx_hal_conf.h"
#include "usart.h"

#define HUART_PRINT (&huart3)
#define HUART_HOST (&huart2)
#define ESP_SPI (&hspi5)

void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif /* __HIVEMIND_HAL_H__ */
