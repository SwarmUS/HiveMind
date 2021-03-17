#include "hal/usb.h"
#include "usbd_cdc_if.h"
#include <USB.h>

bool USB::send(const uint8_t* buffer, uint16_t length) {
    if (buffer == nullptr || length > CBUFF_USB_DATA_SIZE) {
        m_logger.log(LogLevel::Warn, "Invalid parameters for USB::send");
        return false;
    }

    bool ret = false;
    int out = usb_sendData(const_cast<uint8_t*>(buffer), length);

    if (out == USBD_OK) {
        ret = true;
    } else {
        m_logger.log(LogLevel::Warn, "USB_Send_Data was not able to send the data");
    }

    return ret;
}

bool USB::receive(uint8_t* buffer, uint16_t length) {
    if (buffer == nullptr || length > CBUFF_USB_DATA_SIZE) {
        m_logger.log(LogLevel::Warn, "Invalid parameters for USB::Receive");
        return false;
    }

    m_receivingTaskHandle = xTaskGetCurrentTaskHandle();
    while (CircularBuff_getLength(&cbuffUsb) < length) {
        // Gets notified everytime a new packet is appended to cbuffUsb
        ulTaskNotifyTake(pdTRUE, 500);
    }
    m_receivingTaskHandle = NULL;

    CircularBuff_get(&cbuffUsb, buffer, length);

    return true;
}

USB::USB(ILogger& logger) : m_logger(logger) {}

bool USB::isConnected(){
    return usb_isConnected();
}

