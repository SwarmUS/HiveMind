/**
 ******************************************************************************
 * @file    gpio.c
 * @brief   This file provides code for the configuration
 *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
void MX_GPIO_Init(void) {

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, WROOM_EN_Pin | SYNC_Pin | CLK_OE_1__Pin | CLK_OE_0__Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC,
                      MCU_LED_B_Pin | MCU_LED_G_Pin | MCU_LED_1_Pin | MCU_LED_0_Pin |
                          MCU_LED_HB_Pin | CHAN_EN_0B_Pin | SPI_nCS_0B_Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
<<<<<<< HEAD
    HAL_GPIO_WritePin(GPIOF, MCU_LED_R_Pin | MCU_LED_2_Pin, GPIO_PIN_RESET);
=======
    HAL_GPIO_WritePin(GPIOF, MCU_LED_R_Pin | MCU_LED_2_Pin | CHAN_EN_1C_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(PHY_nRST_GPIO_Port, PHY_nRST_Pin, GPIO_PIN_RESET);
>>>>>>> fc1448a... Review/update CubeMX

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, SPI_nCS_1C_Pin | SPI_ESP_nCS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
<<<<<<< HEAD
    HAL_GPIO_WritePin(GPIOD,
                      CHAN_EN_0C_Pin | SPI_nCS_0C_Pin | SPI_nCS_1B_Pin | CHAN_EN_1A_Pin |
                          SPI_nCS_1A_Pin | CHN_RESET_0A_Pin,
                      GPIO_PIN_RESET);
=======
    HAL_GPIO_WritePin(
        GPIOD, CHAN_EN_0C_Pin | SPI_nCS_0C_Pin | SPI_nCS_1B_Pin | CHAN_EN_1A_Pin | SPI_nCS_1A_Pin,
        GPIO_PIN_RESET);
>>>>>>> fc1448a... Review/update CubeMX

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOG, CHAN_EN_1B_Pin | CHAN_EN_0A_Pin | SPI_nCS_0A_Pin | DEBUG_EN__Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pins : PEPin PEPin PEPin PEPin */
    GPIO_InitStruct.Pin = WROOM_EN_Pin | SYNC_Pin | CLK_OE_1__Pin | CLK_OE_0__Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = IMU_INT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(IMU_INT2_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PEPin PEPin */
    GPIO_InitStruct.Pin = USB_PWR_DET__Pin | CHAN_RESET_1C_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : PCPin PCPin PCPin PCPin
                             PCPin PCPin PCPin */
    GPIO_InitStruct.Pin = MCU_LED_B_Pin | MCU_LED_G_Pin | MCU_LED_1_Pin | MCU_LED_0_Pin |
                          MCU_LED_HB_Pin | CHAN_EN_0B_Pin | SPI_nCS_0B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

<<<<<<< HEAD
    /*Configure GPIO pins : PFPin PFPin */
    GPIO_InitStruct.Pin = MCU_LED_R_Pin | MCU_LED_2_Pin;
=======
    /*Configure GPIO pins : PFPin PFPin PFPin */
    GPIO_InitStruct.Pin = MCU_LED_R_Pin | MCU_LED_2_Pin | CHAN_EN_1C_Pin;
>>>>>>> fc1448a... Review/update CubeMX
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

<<<<<<< HEAD
    /*Configure GPIO pins : PAPin PAPin PAPin */
    GPIO_InitStruct.Pin = CHAN_DET_1_C_Pin | CHAN_DET_0B_Pin | CHAN_RESET_0B_Pin;
=======
    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = PHY_nRST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PHY_nRST_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PAPin PAPin PAPin PAPin
                             PAPin */
    GPIO_InitStruct.Pin =
        ETH_PWR_EN_Pin | CH_CL_PWR_EN_Pin | CHAN_DET_1C_Pin | CHAN_DET_0B_Pin | CHAN_RESET_0B_Pin;
>>>>>>> fc1448a... Review/update CubeMX
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = CHAN_IRQ_1C_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CHAN_IRQ_1C_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PBPin PBPin */
    GPIO_InitStruct.Pin = SPI_nCS_1C_Pin | SPI_ESP_nCS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : PDPin PDPin PDPin PDPin
<<<<<<< HEAD
                             PDPin */
    GPIO_InitStruct.Pin =
        CHAN_RESET_0C_Pin | CHAN_DET_0C_Pin | CHAN_RESET_1A_Pin | CHAN_DET_1A_Pin | CHAN_DET_0A_Pin;
=======
                             PDPin PDPin */
    GPIO_InitStruct.Pin = CHAN_RESET_0C_Pin | CHAN_DET_0C_Pin | CHAN_RESET_1A_Pin |
                          CHAN_DET_1A_Pin | CHAN_RESET_0A_Pin | CHAN_DET_0A_Pin;
>>>>>>> fc1448a... Review/update CubeMX
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : PDPin PDPin PDPin PDPin
<<<<<<< HEAD
                             PDPin PDPin */
    GPIO_InitStruct.Pin = CHAN_EN_0C_Pin | SPI_nCS_0C_Pin | SPI_nCS_1B_Pin | CHAN_EN_1A_Pin |
                          SPI_nCS_1A_Pin | CHN_RESET_0A_Pin;
=======
                             PDPin */
    GPIO_InitStruct.Pin =
        CHAN_EN_0C_Pin | SPI_nCS_0C_Pin | SPI_nCS_1B_Pin | CHAN_EN_1A_Pin | SPI_nCS_1A_Pin;
>>>>>>> fc1448a... Review/update CubeMX
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : PDPin PDPin */
    GPIO_InitStruct.Pin = CHAN_IRQ_0C_Pin | CHAN_IRQ_1A_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = CHAN_DET_1B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CHAN_DET_1B_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PGPin PGPin PGPin PGPin */
    GPIO_InitStruct.Pin = CHAN_EN_1B_Pin | CHAN_EN_0A_Pin | SPI_nCS_0A_Pin | DEBUG_EN__Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
<<<<<<< HEAD
    GPIO_InitStruct.Pin = CHAN_IRQ_1B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CHAN_IRQ_1B_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
=======
>>>>>>> fc1448a... Review/update CubeMX
    GPIO_InitStruct.Pin = CHAN_RESET_1B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CHAN_RESET_1B_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
<<<<<<< HEAD
    GPIO_InitStruct.Pin = CHAN_IRQ_0a_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF15_EVENTOUT;
    HAL_GPIO_Init(CHAN_IRQ_0a_GPIO_Port, &GPIO_InitStruct);
=======
    GPIO_InitStruct.Pin = CHAN_IRQ_0B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CHAN_IRQ_0B_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = CHAN_IRQ_0A_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CHAN_IRQ_0A_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PBPin PBPin */
    GPIO_InitStruct.Pin = DEB_PWR_EN_Pin | WROOM_PWR_EN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
>>>>>>> fc1448a... Review/update CubeMX
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
