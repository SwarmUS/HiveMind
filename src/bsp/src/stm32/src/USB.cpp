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
    if(out == 0) {
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

    length = strlen(reinterpret_cast<const char*>(app_data));
    if(length>0) {
        memcpy(buffer,app_data,(uint16_t)(length+(uint16_t)1));
//        USB_Send_Data(const_cast<uint8_t*>(app_data), length);
        USB_rm_data(const_cast<uint8_t*>(app_data));
        m_logger.log(LogLevel::Info, "received data");
    }else{
        m_logger.log(LogLevel::Warn, "no data\r\n");
        return false;
    }

    return true;
}

USB::USB(ILogger& logger):
    m_logger(logger){}

bool USB::isConnected(){
    return USB_isConnected();
}
