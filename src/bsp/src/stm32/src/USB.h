//
// Created by hubert on 3/4/21.
//

#ifndef HIVE_MIND_USB_H
#define HIVE_MIND_USB_H

#include "USB.h"
#include "hal/usb.h"


bool USB::send(const uint8_t* buffer, uint16_t length){
    return USB_Send_Data(buffer, length);
}

bool USB::receive(const uint8_t* buffer, uint16_t length){
    return USB_Send_Data(buffer, length);
}

#endif // HIVE_MIND_USB_H
