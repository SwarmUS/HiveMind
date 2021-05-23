#include "hal/user_interface.h"
#include <hal/hal_gpio.h>

static gpioCallbackFct_t g_buttonCallback;
static void* g_buttonCallbackContext;

void UI_initialize() {}

void UI_setHexOutput(uint8_t hexValue) { (void)hexValue; }

void UI_setButtonCallback(button_t button, gpioCallbackFct_t callback, void* context) {
    // Only one button on the NUCLEO
    if (button == BUTTON_0) {
        g_buttonCallback = callback;
        g_buttonCallbackContext = context;
    }
}

void UI_interruptCallback() {
    // Only one button on the NUCLEO
    if (g_buttonCallback != NULL) {
        g_buttonCallback(g_buttonCallbackContext);
    }
}
