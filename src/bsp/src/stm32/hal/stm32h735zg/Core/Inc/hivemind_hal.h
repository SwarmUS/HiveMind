#ifndef __HIVEMIND_HAL_H__
#define __HIVEMIND_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "crc.h"
#include "gpio.h"
#include "main.h"
#include "rng.h"
#include "spi.h"
#include "stm32h7xx_hal_conf.h"
#include "stm32h7xx_hal_flash.h"
#include "stm32h7xx_hal_flash_ex.h"
#include "stm32h7xx_hal_def.h"
#include "usart.h"

// TODO: change to match requirements
#define HUART_PRINT (&huart3) 
#define HRNG (&hrng)

// Change to ESP_SOC to use th spi channel for the SOC on the HiveSight.
// In the future, there might a flag for the HiveBoard.
#define ESP_WROOM

#ifdef ESP_WROOM
#define ESP_SPI (&hspi3)
#define ESP_USER0_Pin ESP_USER0_WROOM_Pin
#define ESP_USER0_Port ESP_USER0_WROOM_GPIO_Port
#elif ESP_SOC
#define ESP_USER0_Pin ESP_USER0_SOC_Pin
#define ESP_USER0_Port ESP_USER0_SOC_GPIO_Port
#define ESP_SPI (&hspi5)
#endif
#define DW_SPI (&hspi4)

void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif /* __HIVEMIND_HAL_H__ */
