#include "hal/user_interface.h"
#include "tca9539.h"

// https://forum.hosteng.com/wndm/HTMLHelp1/Instruction_Set/SEG_Hex_BCD_to_7_Segment_Display.htm
static uint8_t g_binaryToSegmentMap[16] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,
                                           0x7F, 0x67, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71};

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

    // IO1 has segments G_1 to B_1 on pins 5 to 0
    uint8_t io1 = (g_binaryToSegmentMap[hexDigit1] >> 1) & 0x3F;

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

void UI_setLED(led_t led, bool state) {
    switch (led) {

    case LED_0:
        HAL_GPIO_WritePin(MCU_LED_0_GPIO_Port, MCU_LED_0_Pin,
                          state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;
    case LED_1:
        HAL_GPIO_WritePin(MCU_LED_1_GPIO_Port, MCU_LED_1_Pin,
                          state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;
    case LED_2:
        HAL_GPIO_WritePin(MCU_LED_2_GPIO_Port, MCU_LED_2_Pin,
                          state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;
    case LED_3:
        HAL_GPIO_WritePin(MCU_LED_HB_GPIO_Port, MCU_LED_HB_Pin,
                          state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;
    }
}

void UI_setRGB(bool red, bool green, bool blue) {
    HAL_GPIO_WritePin(MCU_LED_R_GPIO_Port, MCU_LED_R_Pin, red ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MCU_LED_G_GPIO_Port, MCU_LED_G_Pin, green ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MCU_LED_B_GPIO_Port, MCU_LED_B_Pin, blue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
