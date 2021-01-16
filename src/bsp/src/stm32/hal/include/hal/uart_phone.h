#ifndef __UART_PHONE_H__
#define __UART_PHONE_H__

#include "hivemind_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initiates a DMA transfer of the given buffer to the phone UART port
 * @param buffer Pointer to the buffer to transmit
 * @param length Number of bytes to transmit
 * @return True if success. Otherwise, false
 */
bool UartPhone_transmitBuffer(const uint8_t* buffer, uint16_t length);

/**
 * @brief Initiates a DMA reception to the given buffer from the phone UART port
 * @param buffer Pointer to a buffer to store the data into
 * @param length Number of bytes to read
 * @param cpltCallback Function pointer to the callback function, called when the reception is
 * finished
 * @return True if success. Otherwise, false
 */
bool UartPhone_receiveDMA(const uint8_t* buffer, uint16_t length, void (*cpltCallback)());

/**
 * @brief Callback used when reception has finished. Calls the user callback provided in
 * UartPhone_receiveDMA()
 */
void UartPhone_rxCallback();

#ifdef __cplusplus
}
#endif
#endif //__UART_PHONE_H__
