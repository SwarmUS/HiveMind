#include "UserInterface.h"
#include <cstdio>
#include <hal/hal_gpio.h>
#include <hal/user_interface.h>

UserInterface::UserInterface() : m_mutex(10) {}

Mutex& UserInterface::getPrintMutex() { return m_mutex; }

void UserInterface::flush() {
    // Escape character to flush buffer
    printf("\r\n");
}
int UserInterface::print(const char* format, ...) {
    va_list args;
    va_start(args, format);
    int retValue = print(format, args);
    va_end(args);

    return retValue;
}

int UserInterface::print(const char* format, va_list args) {
    int retValue = vprintf(format, args);

    return retValue;
}

int UserInterface::printLine(const char* format, ...) {
    va_list args;
    va_start(args, format);
    int retValue = printLine(format, args);
    va_end(args);

    return retValue;
}

int UserInterface::printLine(const char* format, va_list args) {
    int retValue = print(format, args);
    flush();

    return retValue;
}

void UserInterface::setRGBLed(RgbColor color) {
    switch (color) {
    case RgbColor::RED:
        UI_setRGB(true, false, false);
        break;
    case RgbColor::GREEN:
        UI_setRGB(false, true, false);
        break;
    case RgbColor::BLUE:
        UI_setRGB(false, false, true);
        break;
    case RgbColor::VIOLET:
        UI_setRGB(true, false, true);
        break;
    case RgbColor::YELLOW:
        UI_setRGB(false, true, true);
        break;
    case RgbColor::ORANGE:
        UI_setRGB(true, true, false);
        break;
    case RgbColor::WHITE:
        UI_setRGB(true, true, true);
        break;
    case RgbColor::OFF:
        UI_setRGB(false, false, false);
        break;
    }
}

void UserInterface::setLed(LED led, bool state) {
    led_t halLed;
    switch (led) {
    case LED::LED_0:
        halLed = LED_0;
        break;
    case LED::LED_1:
        halLed = LED_1;
        break;
    case LED::LED_2:
        halLed = LED_2;
        break;
    default:
        return;
    }
    UI_setLED(halLed, state);
}

void UserInterface::setHexDisplay(uint8_t value) { UI_setHexOutput(value); }

void UserInterface::setButtonCallback(Button button,
                                      buttonCallbackFunction_t callback,
                                      void* context) {
    UI_setButtonCallback(static_cast<button_t>(button),
                         reinterpret_cast<gpioCallbackFct_t>(callback), context);
}
