#ifndef HIVE_MIND_HIVEMIND_USB_HAL_H
#define HIVE_MIND_HIVEMIND_USB_HAL_H

#include "usbd_def.h"
#include <stdbool.h>

extern USBD_HandleTypeDef hUsbDeviceHS;
extern bool hUsbDeviceVCPOpened;

#define USB_DEVICE hUsbDeviceHS
#define USB_DEVICE_VPC_OPENED hUsbDeviceVCPOpened
#define USB_TRANSMIT CDC_Transmit_HS

#endif // HIVE_MIND_HIVEMIND_USB_HAL_H