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

void UI_setLED(led_t led, bool state) {
    (void)led;
    (void)state;
}

void UI_setRGB(bool red, bool green, bool blue) {
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD3_Pin, green ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD3_Pin, blue ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, red ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
