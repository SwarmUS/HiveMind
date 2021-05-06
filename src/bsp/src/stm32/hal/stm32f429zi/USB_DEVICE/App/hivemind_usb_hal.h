
#ifndef HIVE_MIND_HIVEMIND_USB_HAL_H
#define HIVE_MIND_HIVEMIND_USB_HAL_H

#include "usbd_def.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
extern bool hUsbDeviceVCPOpened;

#define USB_DEVICE hUsbDeviceFS
#define USB_DEVICE_VPC_OPENED hUsbDeviceVCPOpened
#define USB_TRANSMIT CDC_Transmit_FS

#endif // HIVE_MIND_HIVEMIND_USB_HAL_H
