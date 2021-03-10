// Created by Hubert Dube - 04/03/2021

#ifndef HAL_HIVE_MIND_USB_H
#define HAL_HIVE_MIND_USB_H

#ifdef __cplusplus
extern "C" {
#endif
#include "hivemind_hal.h"

#define CBUFF_USB_DATA_SIZE 2048

CircularBuff cbuffUsb;
extern uint8_t cbuffUsbData[CBUFF_USB_DATA_SIZE];


uint8_t USB_Send_Data(const uint8_t* buf, uint16_t Len);
void USB_rm_data(uint8_t* buf);
bool USB_isConnected();
void Usb_init();

#ifdef __cplusplus
}
#endif

#endif // HIVE_MIND_USB_H
