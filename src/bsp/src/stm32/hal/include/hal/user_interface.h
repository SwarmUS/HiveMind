#ifndef __USER_INTERFACE_H__
#define __USER_INTERFACE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "hal/hal_gpio.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief List of available buttons
 */
typedef enum { BUTTON_0 = 0, BUTTON_1 } button_t;

/**
 * @brief List of available LEDs
 */
typedef enum { LED_0 = 0, LED_1, LED_2, LED_3 } led_t;

/**
 * @brief Initializes any ICs needed for the user interface
 */
void UI_initialize();

/**
 * @brief Sets the callback associated with the press of a given button
 * @param button The button to set the callback for
 * @param callback The callback
 * @param context The context to pass to the callback
 */
void UI_setButtonCallback(button_t button, gpioCallbackFct_t callback, void* context);

/**
 * @brief Function to call when an interrupt happens on a pin associated with the UI
 */
void UI_interruptCallback();

/**
 * @brief Sets the HEX display to the given 8 bit value
 * @param hexValue Value to show on the display
 */
void UI_setHexOutput(uint8_t hexValue);

/**
 * @brief Turns a given LED on or off
 * @param led LED to set
 * @param state State to set (true = ON, false = OFF)
 */
void UI_setLED(led_t led, bool state);

/**
 * @brief Sets the RGB LED (or R, G and B LEDS) to a given pattern
 * @param red Whether to activate red or not
 * @param green Whether to activate green or not
 * @param blue Whether to activate blue or not
 */
void UI_setRGB(bool red, bool green, bool blue);

#ifdef __cplusplus
}
#endif

#endif //__USER_INTERFACE_H__
