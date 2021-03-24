// Created by Hubert Dube - 04/03/2021

#ifndef HAL_HIVE_MIND_USB_H
#define HAL_HIVE_MIND_USB_H

#ifdef __cplusplus
extern "C" {
#endif
#include "hivemind_hal.h"
#include "hivemind_usb_hal.h"

#define CBUFF_USB_DATA_SIZE 2048

extern CircularBuff cbuffUsb;

typedef enum {
    USB_OK = 0U,
    USB_BUSY,
    USB_EMEM,
    USB_FAIL,
} USB_StatusTypeDef;

/**
 * @brief Initiates a data transmission on the USB device.
 *      Will return after the transmission is completed or aborted
 * @param buf Pointer to a buffer to the data to be sent
 * @param len Number of bytes to send
 * @return USB_OK if the data is sent or USB_FAIL in case of errors.
 */
USB_StatusTypeDef usb_sendData(const uint8_t* buf, uint16_t len);

/**
 * @brief Returns the state of the USB device connection
 * True : connected
 * False : not connected
 */
bool usb_isConnected();

/**
 * @brief Initialize the circular buffer for the USB transactions
 */
void usb_init();
void usb_setRxCallback(void (*fct)(void*, uint8_t* buf, uint32_t len), void* context);

#ifdef __cplusplus
}
#endif

#endif // HIVE_MIND_USB_H
