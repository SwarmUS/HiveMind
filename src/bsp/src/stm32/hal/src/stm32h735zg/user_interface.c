#include "hal/user_interface.h"
#include "tca9539.h"

// Hardware lines are in the order (F, G, E, D, A, B, C) (LSB to MSB)
static uint8_t g_binaryToSegmentMap[16] = {0x7D, 0x60, 0x3E, 0x7A, 0x63, 0x5B, 0x5F, 0x70,
                                           0x7F, 0x7B, 0x77, 0x4F, 0x0E, 0x6E, 0x1F, 0x17};

static gpioCallbackFct_t g_buttonCallbacks[2] = {NULL, NULL};
static void* g_buttonCallbackContexts[2] = {NULL, NULL};
static bool g_buttonStates[2] = {true, true};

void UI_initialize() { tca9539_configure(0x00, 0xC0); }

void UI_setHexOutput(uint8_t hexValue) {
    uint8_t hexDigit0 = hexValue & 0x0F;
    uint8_t hexDigit1 = (hexValue >> 4) & 0x0F;

    // IO0 has segment A_1 on pin 7 and G_0 to A_0 on pins 6 to 0
    uint8_t io0 = (((g_binaryToSegmentMap[hexDigit1] << 7) & 0x80) |
                   (g_binaryToSegmentMap[hexDigit0] & 0x7F));
    io0 = ~io0; // LED drive is inverted (1 closes the LED and 0 opens it)

    // IO1 has segments G_1 to B_1 on pins 5 to 0
    uint8_t io1 = g_binaryToSegmentMap[hexDigit1] >> 1;
    io1 = (~io1) & 0x3F; // Invert LED drive and set both buttons to 0

    tca9539_setOutputs(io0, io1);
}

void UI_setButtonCallback(button_t button, gpioCallbackFct_t callback, void* context) {
    if (button <= BUTTON_1) {
        g_buttonCallbacks[button] = callback;
        g_buttonCallbackContexts[button] = context;
    }
}

void UI_interruptCallback() {
    uint8_t io0;
    uint8_t io1;

    // Get state of buttons -> true = released, false = pressed
    tca9539_readInputs(&io0, &io1);
    bool button0State = ((io1 >> 6) & 0x01) == 0x01;
    bool button1State = ((io1 >> 7) & 0x01) == 0x01;

    // Button 0 has been pressed
    if (button0State == false && g_buttonStates[0] == true) {
        if (g_buttonCallbacks[0] != NULL) {
            g_buttonCallbacks[0](g_buttonCallbackContexts[0]);
        }
    }

    // Button 1 has been pressed
    if (button1State == false && g_buttonStates[1] == true) {
        if (g_buttonCallbacks[1] != NULL) {
            g_buttonCallbacks[1](g_buttonCallbackContexts[1]);
        }
    }

    g_buttonStates[0] = button0State;
    g_buttonStates[1] = button1State;
}

void UI_heartbeatCallback() { HAL_GPIO_TogglePin(MCU_LED_HB_GPIO_Port, MCU_LED_HB_Pin); }

void UI_setLED(led_t led, bool state) {
    switch (led) {

    case LED_0:
        HAL_GPIO_WritePin(MCU_LED_0_GPIO_Port, MCU_LED_0_Pin,
                          state ? GPIO_PIN_RESET : GPIO_PIN_SET);
        break;
    case LED_1:
        HAL_GPIO_WritePin(MCU_LED_1_GPIO_Port, MCU_LED_1_Pin,
                          state ? GPIO_PIN_RESET : GPIO_PIN_SET);
        break;
    case LED_2:
        HAL_GPIO_WritePin(MCU_LED_2_GPIO_Port, MCU_LED_2_Pin,
                          state ? GPIO_PIN_RESET : GPIO_PIN_SET);
        break;
    }
}

void UI_setRGB(bool red, bool green, bool blue) {
    HAL_GPIO_WritePin(MCU_LED_R_GPIO_Port, MCU_LED_R_Pin, red ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(MCU_LED_G_GPIO_Port, MCU_LED_G_Pin, green ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(MCU_LED_B_GPIO_Port, MCU_LED_B_Pin, blue ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
