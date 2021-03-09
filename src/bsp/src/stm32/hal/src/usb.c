// Created by Hubert Dube - 04/03/2021

#include "hal/usb.h"
#include "usbd_cdc_if.h"

uint8_t USB_Send_Data(uint8_t* buf, uint16_t Len) {
    return CDC_Transmit_FS(buf, Len);
}

void USB_rm_data(uint8_t* buf) {
    buf[0] = '\0';
}

bool USB_isConnected(){
    return hUsbDeviceFS.dev_connection_status;
}
