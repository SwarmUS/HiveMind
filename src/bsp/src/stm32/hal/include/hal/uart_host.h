#ifndef __UART_HOST_H__
#define __UART_HOST_H__

#include "hivemind_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Typedef for pointer function used as callback for uart operation
 * @param instance Pointer to the C++ class instance in which to call the function
 */
typedef void (*uartCallbackFct)(void* instance);

/**
 * @brief Initiates a DMA transfer of the given buffer to the UART port
 * @param buffer Pointer to the buffer to transmit
 * @param length Number of bytes to transmit
 * @param cpltCallback Function pointer to the callback function, called when the transmission is
 * finished
 * @param instance Pointer to the class instance in which to call the callback
 * @return True if success. Otherwise, false
 */
bool UartHost_transmitBuffer(const uint8_t* buffer,
                             uint16_t length,
                             uartCallbackFct cpltCallback,
                             void* instance);

/**
 * @brief Initiates a DMA reception to the given buffer from the UART port
 * @param buffer Pointer to a buffer to store the data into
 * @param length Number of bytes to read
 * @param cpltCallback Function pointer to the callback function, called when the reception is
 * finished
 * @param instance Pointer to the class instance in which to call the callback
 * @return True if success. Otherwise, false
 */
bool UartHost_receiveDMA(const uint8_t* buffer,
                         uint16_t length,
                         uartCallbackFct cpltCallback,
                         void* instance);

/**
 * @brief Callback used when reception has finished. Calls the user callback provided in
 * UartHost_receiveDMA()
 */
void UartHost_rxCallback();

/**
 * @brief Callback used when transmission has finished. Calls the user callback provided in
 * UartHost_transmitBuffer()
 */
void UartHost_txCallback();

#ifdef __cplusplus
}
#endif
#endif //__UART_HOST_H__
