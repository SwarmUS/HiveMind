//
// Created by hubert on 3/4/21.
//

#include "hal/usb.h"
#include <USB.h>

bool USB::send(const uint8_t* buffer, uint16_t length) {
    USB_Send_Data(const_cast<uint8_t*>(buffer), length);
    return true;
}

bool USB::receive(uint8_t* buffer, uint16_t length) {
    USB_Send_Data(const_cast<uint8_t*>(buffer), length);
    return true;
}
