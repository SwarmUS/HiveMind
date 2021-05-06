#ifndef ESP_SPI_H
#define ESP_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hal/hal_spi.h"
#include "hal_spi.h"
#include "hivemind_hal.h"
#include <stdbool.h>

/**
 * @brief Typedef for pointer function used as callback for spi operation
 * @param instance Pointer to the C++ class instance in which to call the function
 */
typedef void (*spiCallbackFct_t)(void* context);

/**
 * @brief Initiates a DMA transfer of the given buffer to the SPI port of the esp
 * @param buffer Pointer to the buffer to transmit
 * @param length Number of bytes to transmit
 * @param cpltCallback Function pointer to the callback function, called when the transmission is
 * finished
 * @param context Pointer to the class instance in which to call the callback
 * @return True if success. Otherwise, false
 */
bool EspSpi_TransmitDMA(const uint8_t* buffer,
                        uint16_t length,
                        spiCallbackFct_t cpltCallback,
                        void* context);

/**
 * @brief Initiates a DMA reception to the given buffer from the spi port of the esp
 * @param buffer Pointer to a buffer to store the data into
 * @param length Number of bytes to read
 * @param cpltCallback Function pointer to the callback function, called when the reception is
 * finished
 * @param context Pointer to the class instance in which to call the callback
 * @return True if success. Otherwise, false
 */
bool EspSpi_ReceiveDma(const uint8_t* buffer,
                       uint16_t length,
                       spiCallbackFct_t cpltCallback,
                       void* context);

/**
 * @brief Initiates a bidirection DMA transmission and reception to the given buffers from the spi
 * port of the esp
 * @param txBuffer Pointer to the buffer to transmit
 * @param rxBuffer Pointer to a the buffer to store the data into
 * @param lengthBytes Number of bytes to send and receive
 * @param cpltCallback Function pointer to the callback function, called when the reception is
 * finished
 * @param context Pointer to the class instance in which to call the callback
 * @return True if success. Otherwise, false
 */
bool EspSpi_TransmitReceiveDma(const uint8_t* txBuffer,
                               const uint8_t* rxBuffer,
                               uint16_t lengthBytes,
                               spiCallbackFct_t cpltCallback,
                               void* context);

/**
 * @brief Callback used when reception has finished. Calls the user callback provided in
 * espSpiTransmitDma()
 */
void EspSpi_TxCallback();

/**
 * @brief Callback used when reception has finished. Calls the user callback provided in
 * EspSpi_ReceiveDma()
 */
void EspSpi_RxCallback();

/**
 * @brief Callback used when reception has finished. Calls the user callback provided in
 * EspSpi_ReceiveDma()
 */
void EspSpi_TxRxCallback();

#ifdef __cplusplus
}
#endif
#endif // ESP_SPI_H
