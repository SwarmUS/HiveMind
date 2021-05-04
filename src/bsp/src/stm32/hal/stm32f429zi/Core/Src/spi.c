/**
 ******************************************************************************
 * @file    spi.c
 * @brief   This file provides code for the configuration
 *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi5_rx;
DMA_HandleTypeDef hdma_spi5_tx;

/* SPI3 init function */
void MX_SPI3_Init(void) {

    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi3) != HAL_OK) {
        Error_Handler();
    }
}
/* SPI4 init function */
void MX_SPI4_Init(void) {

    hspi4.Instance = SPI4;
    hspi4.Init.Mode = SPI_MODE_MASTER;
    hspi4.Init.Direction = SPI_DIRECTION_2LINES;
    hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi4.Init.NSS = SPI_NSS_SOFT;
    hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi4.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi4) != HAL_OK) {
        Error_Handler();
    }
}
/* SPI5 init function */
void MX_SPI5_Init(void) {

    hspi5.Instance = SPI5;
    hspi5.Init.Mode = SPI_MODE_MASTER;
    hspi5.Init.Direction = SPI_DIRECTION_2LINES;
    hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi5.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi5.Init.NSS = SPI_NSS_SOFT;
    hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi5.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi5) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle) {

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (spiHandle->Instance == SPI3) {
        /* USER CODE BEGIN SPI3_MspInit 0 */

        /* USER CODE END SPI3_MspInit 0 */
        /* SPI3 clock enable */
        __HAL_RCC_SPI3_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**SPI3 GPIO Configuration
        PC10     ------> SPI3_SCK
        PC11     ------> SPI3_MISO
        PC12     ------> SPI3_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* SPI3 DMA Init */
        /* SPI3_RX Init */
        hdma_spi3_rx.Instance = DMA1_Stream0;
        hdma_spi3_rx.Init.Channel = DMA_CHANNEL_0;
        hdma_spi3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_spi3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi3_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi3_rx.Init.Mode = DMA_NORMAL;
        hdma_spi3_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
        hdma_spi3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_spi3_rx) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(spiHandle, hdmarx, hdma_spi3_rx);

        /* SPI3_TX Init */
        hdma_spi3_tx.Instance = DMA1_Stream7;
        hdma_spi3_tx.Init.Channel = DMA_CHANNEL_0;
        hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi3_tx.Init.Mode = DMA_NORMAL;
        hdma_spi3_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
        hdma_spi3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(spiHandle, hdmatx, hdma_spi3_tx);

        /* USER CODE BEGIN SPI3_MspInit 1 */

        /* USER CODE END SPI3_MspInit 1 */
    } else if (spiHandle->Instance == SPI4) {
        /* USER CODE BEGIN SPI4_MspInit 0 */

        /* USER CODE END SPI4_MspInit 0 */
        /* SPI4 clock enable */
        __HAL_RCC_SPI4_CLK_ENABLE();

        __HAL_RCC_GPIOE_CLK_ENABLE();
        /**SPI4 GPIO Configuration
        PE12     ------> SPI4_SCK
        PE13     ------> SPI4_MISO
        PE14     ------> SPI4_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /* USER CODE BEGIN SPI4_MspInit 1 */

        /* USER CODE END SPI4_MspInit 1 */
    } else if (spiHandle->Instance == SPI5) {
        /* USER CODE BEGIN SPI5_MspInit 0 */

        /* USER CODE END SPI5_MspInit 0 */
        /* SPI5 clock enable */
        __HAL_RCC_SPI5_CLK_ENABLE();

        __HAL_RCC_GPIOF_CLK_ENABLE();
        /**SPI5 GPIO Configuration
        PF7     ------> SPI5_SCK
        PF8     ------> SPI5_MISO
        PF9     ------> SPI5_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

        /* SPI5 DMA Init */
        /* SPI5_RX Init */
        hdma_spi5_rx.Instance = DMA2_Stream3;
        hdma_spi5_rx.Init.Channel = DMA_CHANNEL_2;
        hdma_spi5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_spi5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi5_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi5_rx.Init.Mode = DMA_NORMAL;
        hdma_spi5_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
        hdma_spi5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_spi5_rx) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(spiHandle, hdmarx, hdma_spi5_rx);

        /* SPI5_TX Init */
        hdma_spi5_tx.Instance = DMA2_Stream4;
        hdma_spi5_tx.Init.Channel = DMA_CHANNEL_2;
        hdma_spi5_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_spi5_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi5_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi5_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi5_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi5_tx.Init.Mode = DMA_NORMAL;
        hdma_spi5_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
        hdma_spi5_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_spi5_tx) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(spiHandle, hdmatx, hdma_spi5_tx);

        /* SPI5 interrupt Init */
        HAL_NVIC_SetPriority(SPI5_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(SPI5_IRQn);
        /* USER CODE BEGIN SPI5_MspInit 1 */

        /* USER CODE END SPI5_MspInit 1 */
    }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle) {

    if (spiHandle->Instance == SPI3) {
        /* USER CODE BEGIN SPI3_MspDeInit 0 */

        /* USER CODE END SPI3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_SPI3_CLK_DISABLE();

        /**SPI3 GPIO Configuration
        PC10     ------> SPI3_SCK
        PC11     ------> SPI3_MISO
        PC12     ------> SPI3_MOSI
        */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);

        /* SPI3 DMA DeInit */
        HAL_DMA_DeInit(spiHandle->hdmarx);
        HAL_DMA_DeInit(spiHandle->hdmatx);
        /* USER CODE BEGIN SPI3_MspDeInit 1 */

        /* USER CODE END SPI3_MspDeInit 1 */
    } else if (spiHandle->Instance == SPI4) {
        /* USER CODE BEGIN SPI4_MspDeInit 0 */

        /* USER CODE END SPI4_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_SPI4_CLK_DISABLE();

        /**SPI4 GPIO Configuration
        PE12     ------> SPI4_SCK
        PE13     ------> SPI4_MISO
        PE14     ------> SPI4_MOSI
        */
        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14);

        /* USER CODE BEGIN SPI4_MspDeInit 1 */

        /* USER CODE END SPI4_MspDeInit 1 */
    } else if (spiHandle->Instance == SPI5) {
        /* USER CODE BEGIN SPI5_MspDeInit 0 */

        /* USER CODE END SPI5_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_SPI5_CLK_DISABLE();

        /**SPI5 GPIO Configuration
        PF7     ------> SPI5_SCK
        PF8     ------> SPI5_MISO
        PF9     ------> SPI5_MOSI
        */
        HAL_GPIO_DeInit(GPIOF, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);

        /* SPI5 DMA DeInit */
        HAL_DMA_DeInit(spiHandle->hdmarx);
        HAL_DMA_DeInit(spiHandle->hdmatx);

        /* SPI5 interrupt Deinit */
        HAL_NVIC_DisableIRQ(SPI5_IRQn);
        /* USER CODE BEGIN SPI5_MspDeInit 1 */

        /* USER CODE END SPI5_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
