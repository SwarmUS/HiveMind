/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define WROOM_EN_Pin GPIO_PIN_2
#define WROOM_EN_GPIO_Port GPIOE
#define IMU_INT2_Pin GPIO_PIN_4
#define IMU_INT2_GPIO_Port GPIOE
#define USB_PWR_DET__Pin GPIO_PIN_5
#define USB_PWR_DET__GPIO_Port GPIOE
#define MCU_LED_B_Pin GPIO_PIN_14
#define MCU_LED_B_GPIO_Port GPIOC
#define MCU_LED_G_Pin GPIO_PIN_15
#define MCU_LED_G_GPIO_Port GPIOC
#define MCU_LED_R_Pin GPIO_PIN_6
#define MCU_LED_R_GPIO_Port GPIOF
#define ESP_SPI_CLK_Pin GPIO_PIN_7
#define ESP_SPI_CLK_GPIO_Port GPIOF
#define ESP_SPI_MISO_Pin GPIO_PIN_8
#define ESP_SPI_MISO_GPIO_Port GPIOF
#define ESP_SPI_MOSI_Pin GPIO_PIN_9
#define ESP_SPI_MOSI_GPIO_Port GPIOF
#define MCU_LED_2_Pin GPIO_PIN_10
#define MCU_LED_2_GPIO_Port GPIOF
#define MCU_LED_1_Pin GPIO_PIN_0
#define MCU_LED_1_GPIO_Port GPIOC
#define MCU_LED_0_Pin GPIO_PIN_2
#define MCU_LED_0_GPIO_Port GPIOC
#define MCU_LED_HB_Pin GPIO_PIN_3
#define MCU_LED_HB_GPIO_Port GPIOC
#define PHY_nRST_Pin GPIO_PIN_0
#define PHY_nRST_GPIO_Port GPIOA
#define ETH_PWR_EN_Pin GPIO_PIN_3
#define ETH_PWR_EN_GPIO_Port GPIOA
#define CH_CL_PWR_EN_Pin GPIO_PIN_4
#define CH_CL_PWR_EN_GPIO_Port GPIOA
#define CHAN_DET_1C_Pin GPIO_PIN_5
#define CHAN_DET_1C_GPIO_Port GPIOA
#define CHAN_IRQ_1C_Pin GPIO_PIN_6
#define CHAN_IRQ_1C_GPIO_Port GPIOA
#define MEZ_ADC1_N_Pin GPIO_PIN_0
#define MEZ_ADC1_N_GPIO_Port GPIOB
#define MEZ_ADC1_P_Pin GPIO_PIN_1
#define MEZ_ADC1_P_GPIO_Port GPIOB
#define SPI_nCS_1C_Pin GPIO_PIN_2
#define SPI_nCS_1C_GPIO_Port GPIOB
#define CHAN_EN_1C_Pin GPIO_PIN_11
#define CHAN_EN_1C_GPIO_Port GPIOF
#define MEZ_I2C_SCL_Pin GPIO_PIN_14
#define MEZ_I2C_SCL_GPIO_Port GPIOF
#define MEZ_I2C_SDA_Pin GPIO_PIN_15
#define MEZ_I2C_SDA_GPIO_Port GPIOF
#define MEZ_UART_RX_Pin GPIO_PIN_7
#define MEZ_UART_RX_GPIO_Port GPIOE
#define MEZ_UART_TX_Pin GPIO_PIN_8
#define MEZ_UART_TX_GPIO_Port GPIOE
#define SYNC_Pin GPIO_PIN_9
#define SYNC_GPIO_Port GPIOE
#define CLK_OE_1__Pin GPIO_PIN_10
#define CLK_OE_1__GPIO_Port GPIOE
#define CLK_OE_0__Pin GPIO_PIN_11
#define CLK_OE_0__GPIO_Port GPIOE
#define MEZ_SPI_SCK_Pin GPIO_PIN_12
#define MEZ_SPI_SCK_GPIO_Port GPIOE
#define MEZ_SPI_MISO_Pin GPIO_PIN_13
#define MEZ_SPI_MISO_GPIO_Port GPIOE
#define MEZ_SPI_MOSI_Pin GPIO_PIN_14
#define MEZ_SPI_MOSI_GPIO_Port GPIOE
#define CHAN_RESET_1C_Pin GPIO_PIN_15
#define CHAN_RESET_1C_GPIO_Port GPIOE
#define SPI_CLK_C_Pin GPIO_PIN_10
#define SPI_CLK_C_GPIO_Port GPIOB
#define SPI_MISO_C_Pin GPIO_PIN_14
#define SPI_MISO_C_GPIO_Port GPIOB
#define SPI_MOSI_C_Pin GPIO_PIN_15
#define SPI_MOSI_C_GPIO_Port GPIOB
#define STM_UTx_Pin GPIO_PIN_8
#define STM_UTx_GPIO_Port GPIOD
#define STM_URx_Pin GPIO_PIN_9
#define STM_URx_GPIO_Port GPIOD
#define CHAN_RESET_0C_Pin GPIO_PIN_10
#define CHAN_RESET_0C_GPIO_Port GPIOD
#define CHAN_DET_0C_Pin GPIO_PIN_11
#define CHAN_DET_0C_GPIO_Port GPIOD
#define CHAN_EN_0C_Pin GPIO_PIN_12
#define CHAN_EN_0C_GPIO_Port GPIOD
#define CHAN_IRQ_0C_Pin GPIO_PIN_13
#define CHAN_IRQ_0C_GPIO_Port GPIOD
#define SPI_nCS_0C_Pin GPIO_PIN_14
#define SPI_nCS_0C_GPIO_Port GPIOD
#define SPI_nCS_1B_Pin GPIO_PIN_15
#define SPI_nCS_1B_GPIO_Port GPIOD
#define CHAN_DET_1B_Pin GPIO_PIN_6
#define CHAN_DET_1B_GPIO_Port GPIOG
#define CHAN_EN_1B_Pin GPIO_PIN_7
#define CHAN_EN_1B_GPIO_Port GPIOG
#define CHAN_RESET_1B_Pin GPIO_PIN_6
#define CHAN_RESET_1B_GPIO_Port GPIOC
#define CHAN_EN_0B_Pin GPIO_PIN_7
#define CHAN_EN_0B_GPIO_Port GPIOC
#define CHAN_IRQ_0B_Pin GPIO_PIN_8
#define CHAN_IRQ_0B_GPIO_Port GPIOC
#define SPI_nCS_0B_Pin GPIO_PIN_9
#define SPI_nCS_0B_GPIO_Port GPIOC
#define CHAN_DET_0B_Pin GPIO_PIN_8
#define CHAN_DET_0B_GPIO_Port GPIOA
#define CHAN_RESET_0B_Pin GPIO_PIN_10
#define CHAN_RESET_0B_GPIO_Port GPIOA
#define SPI_CLK_B_Pin GPIO_PIN_10
#define SPI_CLK_B_GPIO_Port GPIOC
#define SPI_MISO_CC11_Pin GPIO_PIN_11
#define SPI_MISO_CC11_GPIO_Port GPIOC
#define SPI_MOSI_B_Pin GPIO_PIN_12
#define SPI_MOSI_B_GPIO_Port GPIOC
#define CHAN_RESET_1A_Pin GPIO_PIN_0
#define CHAN_RESET_1A_GPIO_Port GPIOD
#define CHAN_DET_1A_Pin GPIO_PIN_1
#define CHAN_DET_1A_GPIO_Port GPIOD
#define CHAN_EN_1A_Pin GPIO_PIN_2
#define CHAN_EN_1A_GPIO_Port GPIOD
#define CHAN_IRQ_1A_Pin GPIO_PIN_3
#define CHAN_IRQ_1A_GPIO_Port GPIOD
#define SPI_nCS_1A_Pin GPIO_PIN_4
#define SPI_nCS_1A_GPIO_Port GPIOD
#define CHAN_RESET_0A_Pin GPIO_PIN_5
#define CHAN_RESET_0A_GPIO_Port GPIOD
#define CHAN_DET_0A_Pin GPIO_PIN_6
#define CHAN_DET_0A_GPIO_Port GPIOD
#define SPI_MOSI_A_Pin GPIO_PIN_7
#define SPI_MOSI_A_GPIO_Port GPIOD
#define SPI_MISO_A_Pin GPIO_PIN_9
#define SPI_MISO_A_GPIO_Port GPIOG
#define CHAN_IRQ_0A_Pin GPIO_PIN_10
#define CHAN_IRQ_0A_GPIO_Port GPIOG
#define SPI_CLK_A_Pin GPIO_PIN_11
#define SPI_CLK_A_GPIO_Port GPIOG
#define CHAN_EN_0A_Pin GPIO_PIN_12
#define CHAN_EN_0A_GPIO_Port GPIOG
#define SPI_nCS_0A_Pin GPIO_PIN_13
#define SPI_nCS_0A_GPIO_Port GPIOG
#define DEBUG_EN__Pin GPIO_PIN_14
#define DEBUG_EN__GPIO_Port GPIOG
#define DEB_PWR_EN_Pin GPIO_PIN_5
#define DEB_PWR_EN_GPIO_Port GPIOB
#define I2C_MISC_SCL_Pin GPIO_PIN_6
#define I2C_MISC_SCL_GPIO_Port GPIOB
#define IMU_MISC_SDA_Pin GPIO_PIN_7
#define IMU_MISC_SDA_GPIO_Port GPIOB
#define SPI_ESP_nCS_Pin GPIO_PIN_8
#define SPI_ESP_nCS_GPIO_Port GPIOB
#define WROOM_PWR_EN_Pin GPIO_PIN_9
#define WROOM_PWR_EN_GPIO_Port GPIOB
#define UART_WROOM_RX_Pin GPIO_PIN_0
#define UART_WROOM_RX_GPIO_Port GPIOE
#define UART_WROOM_TX_Pin GPIO_PIN_1
#define UART_WROOM_TX_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
