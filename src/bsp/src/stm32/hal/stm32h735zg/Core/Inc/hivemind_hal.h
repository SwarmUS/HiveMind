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
#include "stm32h7xx_hal_conf.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_flash.h"
#include "stm32h7xx_hal_flash_ex.h"
#include "usart.h"

// TODO: Temporary so dw and esp builds
#define ESP_CS_Pin GPIO_PIN_0
#define ESP_CS_GPIO_Port GPIOC
#define DW_IRQn_B_Pin GPIO_PIN_15
#define DW_IRQn_B_GPIO_Port GPIOF
#define DW_IRQn_B_EXTI_IRQn EXTI15_10_IRQn
#define ESP_USER0_SOC_Pin GPIO_PIN_1
#define ESP_USER0_SOC_GPIO_Port GPIOG
#define ESP_USER0_SOC_EXTI_IRQn EXTI1_IRQn
#define DW_IRQn_A_Pin GPIO_PIN_7
#define DW_IRQn_A_GPIO_Port GPIOE
#define DW_IRQn_A_EXTI_IRQn EXTI9_5_IRQn
#define DW_WU_A_Pin GPIO_PIN_8
#define DW_WU_A_GPIO_Port GPIOE
#define DW_NSS_A_Pin GPIO_PIN_11
#define DW_NSS_A_GPIO_Port GPIOE
#define DW_NSS_B_Pin GPIO_PIN_15
#define DW_NSS_B_GPIO_Port GPIOE
#define DW_RESET_A_Pin GPIO_PIN_10
#define DW_RESET_A_GPIO_Port GPIOB
#define DW_RESET_B_Pin GPIO_PIN_11
#define DW_RESET_B_GPIO_Port GPIOB
#define DW_SYNC_Pin GPIO_PIN_3
#define DW_SYNC_GPIO_Port GPIOD
#define DW_SYNC_CLEAR_Pin GPIO_PIN_4
#define DW_SYNC_CLEAR_GPIO_Port GPIOD
#define DW_SYNC_EN_Pin GPIO_PIN_5
#define DW_SYNC_EN_GPIO_Port GPIOD
#define DW_EXTON_A_Pin GPIO_PIN_9
#define DW_EXTON_A_GPIO_Port GPIOG

// TODO: change to match requirements
#define HUART_PRINT (&huart3)
#define HRNG (&hrng)

// Change to ESP_SOC to use th spi channel for the SOC on the HiveSight.
// In the future, there might a flag for the HiveBoard.
#define ESP_WROOM

#ifdef ESP_WROOM
#define ESP_SPI (&hspi3)
#define ESP_USER0_Pin 0
#define ESP_USER0_Port 0
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
