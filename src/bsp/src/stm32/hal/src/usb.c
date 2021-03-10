// Created by Hubert Dube - 04/03/2021

#include "hal/usb.h"
#include "usbd_cdc_if.h"

uint8_t cbuffUsbData[CBUFF_USB_DATA_SIZE];

uint8_t USB_Send_Data(const uint8_t* buf, uint16_t Len) {
    CDC_Transmit_FS((uint8_t*)buf, Len);
    USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    while(hcdc->TxState != 0){
    }
    return USBD_OK; // to change
}

void USB_rm_data(uint8_t* buf) {
    buf[0] = '\0';
}

bool USB_isConnected(){
    return hUsbDeviceFS.dev_connection_status;
}

void Usb_init() {
    CircularBuff_init(&cbuffUsb, cbuffUsbData, CBUFF_USB_DATA_SIZE);
}

void USB_CDC_RxCallBack(uint8_t* Buf, uint32_t Len) {
    CircularBuff_put(&cbuffUsb, Buf,Len);
}
