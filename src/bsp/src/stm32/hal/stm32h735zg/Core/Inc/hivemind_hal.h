#ifndef __HIVEMIND_HAL_H__
#define __HIVEMIND_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "crc.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "main.h"
#include "rng.h"
#include "spi.h"
#include "stm32h7xx_hal_conf.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_flash.h"
#include "stm32h7xx_hal_flash_ex.h"
#include "usart.h"

// TODO: change to match requirements
#define HUART_PRINT (&huart3)
#define HRNG (&hrng)
#define FLASH_PROGRAM_32_BYTES (FLASH_TYPEPROGRAM_FLASHWORD)

#define UI_INTERRUPT_Pin IO_EXPANDER_INT__Pin
#define UI_INTERRUPT_Port IO_EXPANDER_INT__Port

#define ESP_SPI (&hspi5)
#define ESP_USER0_Pin WROOM_INT_Pin
#define ESP_USER0_Port WROOM_INT_GPIO_Port
#define ESP_CS_GPIO_Port SPI_ESP_nCS_GPIO_Port
#define ESP_CS_Pin SPI_ESP_nCS_Pin

#define IO_EXPANDER_I2C (&hi2c1)

void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif /* __HIVEMIND_HAL_H__ */
