#ifndef __HIVEMIND_HAL_H__
#define __HIVEMIND_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "crc.h"
#include "dma.h"
#include "gpio.h"
#include "main.h"
#include "rng.h"
#include "spi.h"
#include "stm32f4xx_hal_conf.h"
#include "usart.h"

#define HUART_PRINT (&huart3)
#define HUART_HOST (&huart2)
#define HRNG (&hrng)

// Change to ESP_SOC to use th spi channel for the SOC on the HiveSight.
// In the future, there might a flag for the HiveBoard.
#define ESP_WROOM

#ifdef ESP_WROOM
#define ESP_SPI (&hspi3)
#elif ESP_SOC
#define ESP_SPI (&hspi5)
#endif

void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif /* __HIVEMIND_HAL_H__ */
