/**
 ******************************************************************************
 * @file    usart.c
 * @brief   This file provides code for the configuration
 *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart3;

/* UART7 init function */
void MX_UART7_Init(void) {

    /* USER CODE BEGIN UART7_Init 0 */

    /* USER CODE END UART7_Init 0 */

    /* USER CODE BEGIN UART7_Init 1 */

    /* USER CODE END UART7_Init 1 */
    huart7.Instance = UART7;
    huart7.Init.BaudRate = 115200;
    huart7.Init.WordLength = UART_WORDLENGTH_8B;
    huart7.Init.StopBits = UART_STOPBITS_1;
    huart7.Init.Parity = UART_PARITY_NONE;
    huart7.Init.Mode = UART_MODE_TX_RX;
    huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart7.Init.OverSampling = UART_OVERSAMPLING_16;
    huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart7) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN UART7_Init 2 */

    /* USER CODE END UART7_Init 2 */
}
/* UART8 init function */
void MX_UART8_Init(void) {

    /* USER CODE BEGIN UART8_Init 0 */

    /* USER CODE END UART8_Init 0 */

    /* USER CODE BEGIN UART8_Init 1 */

    /* USER CODE END UART8_Init 1 */
    huart8.Instance = UART8;
    huart8.Init.BaudRate = 115200;
    huart8.Init.WordLength = UART_WORDLENGTH_8B;
    huart8.Init.StopBits = UART_STOPBITS_1;
    huart8.Init.Parity = UART_PARITY_NONE;
    huart8.Init.Mode = UART_MODE_TX_RX;
    huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart8.Init.OverSampling = UART_OVERSAMPLING_16;
    huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart8.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart8, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart8, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart8) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN UART8_Init 2 */

    /* USER CODE END UART8_Init 2 */
}
/* USART3 init function */

void MX_USART3_UART_Init(void) {

    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle) {

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if (uartHandle->Instance == UART7) {
        /* USER CODE BEGIN UART7_MspInit 0 */

        /* USER CODE END UART7_MspInit 0 */
        /** Initializes the peripherals clock
         */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART7;
        PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
            Error_Handler();
        }

        /* UART7 clock enable */
        __HAL_RCC_UART7_CLK_ENABLE();

        __HAL_RCC_GPIOE_CLK_ENABLE();
        /**UART7 GPIO Configuration
        PE7     ------> UART7_RX
        PE8     ------> UART7_TX
        */
        GPIO_InitStruct.Pin = MEZ_UART_RX_Pin | MEZ_UART_TX_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_UART7;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /* USER CODE BEGIN UART7_MspInit 1 */

        /* USER CODE END UART7_MspInit 1 */
    } else if (uartHandle->Instance == UART8) {
        /* USER CODE BEGIN UART8_MspInit 0 */

        /* USER CODE END UART8_MspInit 0 */

        /** Initializes the peripherals clock
         */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART8;
        PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
            Error_Handler();
        }

        /* UART8 clock enable */
        __HAL_RCC_UART8_CLK_ENABLE();

        __HAL_RCC_GPIOE_CLK_ENABLE();
        /**UART8 GPIO Configuration
        PE0     ------> UART8_RX
        PE1     ------> UART8_TX
        */
        GPIO_InitStruct.Pin = UART_WROOM_RX_Pin | UART_WROOM_TX_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART8;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /* USER CODE BEGIN UART8_MspInit 1 */

        /* USER CODE END UART8_MspInit 1 */
    } else if (uartHandle->Instance == USART3) {
        /* USER CODE BEGIN USART3_MspInit 0 */

        /* USER CODE END USART3_MspInit 0 */

        /** Initializes the peripherals clock
         */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
        PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
            Error_Handler();
        }

        /* USART3 clock enable */
        __HAL_RCC_USART3_CLK_ENABLE();

        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**USART3 GPIO Configuration
        PD8     ------> USART3_TX
        PD9     ------> USART3_RX
        */
        GPIO_InitStruct.Pin = STM_UTx_Pin | STM_URx_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* USART3 interrupt Init */
        HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
        /* USER CODE BEGIN USART3_MspInit 1 */

        /* USER CODE END USART3_MspInit 1 */
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle) {

    if (uartHandle->Instance == UART7) {
        /* USER CODE BEGIN UART7_MspDeInit 0 */

        /* USER CODE END UART7_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_UART7_CLK_DISABLE();

        /**UART7 GPIO Configuration
        PE7     ------> UART7_RX
        PE8     ------> UART7_TX
        */
        HAL_GPIO_DeInit(GPIOE, MEZ_UART_RX_Pin | MEZ_UART_TX_Pin);

        /* USER CODE BEGIN UART7_MspDeInit 1 */

        /* USER CODE END UART7_MspDeInit 1 */
    } else if (uartHandle->Instance == UART8) {
        /* USER CODE BEGIN UART8_MspDeInit 0 */

        /* USER CODE END UART8_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_UART8_CLK_DISABLE();

        /**UART8 GPIO Configuration
        PE0     ------> UART8_RX
        PE1     ------> UART8_TX
        */
        HAL_GPIO_DeInit(GPIOE, UART_WROOM_RX_Pin | UART_WROOM_TX_Pin);

        /* USER CODE BEGIN UART8_MspDeInit 1 */

        /* USER CODE END UART8_MspDeInit 1 */
    } else if (uartHandle->Instance == USART3) {
        /* USER CODE BEGIN USART3_MspDeInit 0 */

        /* USER CODE END USART3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART3_CLK_DISABLE();

        /**USART3 GPIO Configuration
        PD8     ------> USART3_TX
        PD9     ------> USART3_RX
        */
        HAL_GPIO_DeInit(GPIOD, STM_UTx_Pin | STM_URx_Pin);

        /* USART3 interrupt Deinit */
        HAL_NVIC_DisableIRQ(USART3_IRQn);
        /* USER CODE BEGIN USART3_MspDeInit 1 */

        /* USER CODE END USART3_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
