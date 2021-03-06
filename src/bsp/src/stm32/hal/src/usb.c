#include "hal/usb.h"
#include "usbd_cdc_if.h"

CircularBuff cbuffUsb;
uint8_t cbuffUsbData[CBUFF_USB_DATA_SIZE];
void (*usb_rxCallback)();
void* usb_rxCallbackContext;

USB_StatusTypeDef usb_hasTxFinished(USBD_CDC_HandleTypeDef* hcdc) {
    while (hcdc->TxState != 0) {
    }
    return USB_OK;
}

USB_StatusTypeDef usb_sendData(const uint8_t* buf, uint16_t Len) {
    uint8_t status = USB_TRANSMIT((uint8_t*)buf, Len);

    if (status != USBD_OK) {
        return USB_FAIL;
    }

    //    TODO : add the data in another output queue so it's non blocking
    USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)USB_DEVICE.pClassData;
    return usb_hasTxFinished(hcdc);
}

bool usb_isConnected() {
    // If device was disconnected without proper closing, the VCP will still be considered opened so
    // we set it to closed if the wire is disconnected.
    USB_DEVICE_VPC_OPENED = USB_DEVICE.dev_state == USBD_STATE_CONFIGURED && USB_DEVICE_VPC_OPENED;
    return USB_DEVICE_VPC_OPENED;
}

void usb_init() { CircularBuff_init(&cbuffUsb, cbuffUsbData, CBUFF_USB_DATA_SIZE); }

void usb_CDC_rxCallBack(uint8_t* buf, uint32_t len) {
    usb_rxCallback(usb_rxCallbackContext, buf, len);
}

void usb_setRxCallback(void (*fct)(void* context, uint8_t* buf, uint32_t len), void* context) {
    usb_rxCallback = fct;
    usb_rxCallbackContext = context;
}
