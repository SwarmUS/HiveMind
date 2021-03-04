// Created by Hubert Dube - 04/03/2021

#ifndef HAL_HIVE_MIND_USB_H
#define HAL_HIVE_MIND_USB_H

#ifdef __cplusplus
extern "C" {
#endif
#include "hivemind_hal.h"

uint8_t USB_Send_Data(uint8_t* buf, uint16_t Len);

#ifdef __cplusplus
}
#endif

#endif // HIVE_MIND_USB_H
