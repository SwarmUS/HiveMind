//
// Created by hubert on 3/4/21.
//

#include "hal/usb.h"
#include "usbd_cdc_if.h"
#include <USB.h>

bool USB::send(const uint8_t* buffer, uint16_t length) {
    if (buffer == nullptr || length > sizeof(app_data)) {
        m_logger.log(LogLevel::Warn, "Invalid parameters for USB::send");
        return false;
    }

    bool ret = false;
    int out = USB_Send_Data(const_cast<uint8_t*>(buffer), length);

    if(out == USBD_OK) {
        ret = true;
    }else{
        m_logger.log(LogLevel::Warn, "USB_Send_Data was not able to send the data");
    }

    return ret;
}

bool USB::receive(uint8_t* buffer, uint16_t length) {
    if (buffer == nullptr || length > sizeof(app_data)) {
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
    m_logger.log(LogLevel::Info, "received data");

    return true;
}

USB::USB(ILogger& logger):
    m_logger(logger){}

bool USB::isConnected(){
    return USB_isConnected();
}
