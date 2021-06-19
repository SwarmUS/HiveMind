#include "hal/hal_timer.h"
#include "hal/user_interface.h"
#include <TestUI.h>

void TestUI::runTests() {
    setup();

    test01_Heartbeat();
    test02_Leds();
    test03_RGB();
    test04_Buttons();
    test05_HexDisplay();
}

void TestUI::setup() { UI_initialize(); }

void TestUI::test01_Heartbeat() { Timer_startHeartbeat(); }

void TestUI::test02_Leds() {
    UI_setLED(LED_0, true);
    UI_setLED(LED_0, false);

    UI_setLED(LED_1, true);
    UI_setLED(LED_1, false);

    UI_setLED(LED_2, true);
    UI_setLED(LED_2, false);
}

void TestUI::test03_RGB() {
    // RED
    UI_setRGB(true, false, false);

    // GREEN
    UI_setRGB(false, true, false);

    // BLUE
    UI_setRGB(false, false, true);

    // ALL
    UI_setRGB(true, true, true);

    // NONE
    UI_setRGB(false, false, false);
}

void TestUI::test04_Buttons() {
    UI_setButtonCallback(BUTTON_0, button0Callback, this);
    UI_setButtonCallback(BUTTON_1, button1Callback, this);

    // Press button 0 to continue
    while (!button0Pressed) {
        HAL_Delay(100);
    }

    // Press button 1 to continue
    while (!button1Pressed) {
        HAL_Delay(100);
    }
}

void TestUI::test05_HexDisplay() {
    // DIGIT 1
    for (uint8_t i = 0; i < 16; i++) {
        UI_setHexOutput(i);
    }

    // DIGIT 2
    for (uint8_t i = 0; i < 16; i++) {
        UI_setHexOutput((i << 4) & 0xF0);
    }
}

void TestUI::button0Callback(void* context) {
    static_cast<TestUI*>(context)->button0Pressed = true;
}

void TestUI::button1Callback(void* context) {
    static_cast<TestUI*>(context)->button1Pressed = true;
}
