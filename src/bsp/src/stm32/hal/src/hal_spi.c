#include "hal/esp_spi.h"

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi == ESP_SPI) {
        EspSpi_TxCallback();
    }
    // Future code for Decawave should go there
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi == ESP_SPI) {
        EspSpi_RxCallback();
    }
    // Future code for Decawave should go there
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi == ESP_SPI) {
        EspSpi_TxRxCallback();
    }
    // Future code for Decawave should go there
}