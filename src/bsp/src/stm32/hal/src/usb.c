#include "hal/usb.h"
#include "usbd_cdc_if.h"

uint8_t cbuffUsbData[CBUFF_USB_DATA_SIZE];
USB_StatusTypeDef usb_hasTxFinished(USBD_CDC_HandleTypeDef* hcdc){
    while (hcdc->TxState != 0) {
    }
    return USB_OK;
}

USB_StatusTypeDef usb_sendData(const uint8_t* buf, uint16_t Len) {
    CDC_Transmit_FS((uint8_t*)buf, Len);
//    TODO : add the data in another output queue so it's non blocking
    USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    while (hcdc->TxState != 0) {
    }

    return usb_hasTxFinished(hcdc);
}

bool usb_isConnected(){
    return hUsbDeviceFS.dev_connection_status;
}

void usb_init() {
    CircularBuff_init(&cbuffUsb, cbuffUsbData, CBUFF_USB_DATA_SIZE);
}

void usb_CDC_rxCallBack(uint8_t* buf, uint32_t len) {
    if(CircularBuff_getLength(&cbuffUsb) + len  > CBUFF_USB_DATA_SIZE){
        // TODO should notify the user of an error
        CircularBuff_clear(&cbuffUsb);
        return;
    }
    CircularBuff_put(&cbuffUsb, buf, len);
}
