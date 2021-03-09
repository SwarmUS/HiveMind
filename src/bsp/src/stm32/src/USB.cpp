//
// Created by hubert on 3/4/21.
//

#include "hal/usb.h"
#include "usbd_cdc_if.h"
#include <USB.h>

bool USB::send(const uint8_t* buffer, uint16_t length) {
    bool ret = false;
    int out = USB_Send_Data(const_cast<uint8_t*>(buffer), length);
    if(out == 0) {
        ret = true;
    }
    return ret;
}

bool USB::receive(uint8_t* buffer, uint16_t length) {
    buffer = app_data;
    length = strlen(reinterpret_cast<const char*>(app_data));
    if(length>0) {
        USB_Send_Data(const_cast<uint8_t*>(buffer), length);
        USB_rm_data(const_cast<uint8_t*>(buffer));
    }

    return true;
}
