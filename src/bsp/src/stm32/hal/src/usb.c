#include "hal/usb.h"
#include "usbd_cdc_if.h"

uint8_t cbuffUsbData[CBUFF_USB_DATA_SIZE];

uint8_t Usb_HasTxFinished(USBD_CDC_HandleTypeDef* hcdc){
    bool ret = false;
    while(hcdc->TxState != 0){}
    if (hcdc->TxState == 0) {
        ret = USBD_OK;
    }else{
        ret = USBD_FAIL;
    }

    return ret;
}

uint8_t Usb_Send_Data(const uint8_t* buf, uint16_t Len) {
    CDC_Transmit_FS((uint8_t*)buf, Len);
    USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    while(hcdc->TxState != 0){}

    return Usb_HasTxFinished(hcdc);
}

bool Usb_isConnected(){
    return hUsbDeviceFS.dev_connection_status;
}

void Usb_init() {
    CircularBuff_init(&cbuffUsb, cbuffUsbData, CBUFF_USB_DATA_SIZE);
}

void Usb_CDC_RxCallBack(uint8_t* Buf, uint32_t len) {
    if(CircularBuff_getLength(&cbuffUsb) + len  > CBUFF_USB_DATA_SIZE){
        // TODO should notify the user of an error
        CircularBuff_clear(&cbuffUsb);
        return;
    }
    CircularBuff_put(&cbuffUsb, Buf,len);
}
