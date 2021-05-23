#ifndef __USER_INTERFACE_H__
#define __USER_INTERFACE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "hal/hal_gpio.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum { BUTTON_0 = 0, BUTTON_1 } button_t;
typedef enum { LED_0 = 0, LED_1, LED_2, LED_3 } led_t;

void UI_initialize();

void UI_setButtonCallback(button_t button, gpioCallbackFct_t callback, void* context);

void UI_interruptCallback();

void UI_setHexOutput(uint8_t hexValue);

void UI_setLED(led_t led, bool state);

void UI_setRGB(bool red, bool green, bool blue);

#ifdef __cplusplus
}
#endif

#endif //__USER_INTERFACE_H__
