#include "UserInterface.h"
#include <cstdint>
#include <cstdio>
#include <hal/hal_gpio.h>
#include <hal/user_interface.h>
#include <hivemind_hal.h>

UserInterface::UserInterface() : m_mutex(10) { UI_initialize(); }

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
void UserInterface::setLedState(Led led, bool state) {}

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
    case RgbColor::BROWN:
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

void UserInterface::setHexDisplay(uint8_t value) { UI_setHexOutput(value); }

void UserInterface::setButtonCallback(Button button,
                                      buttonCallbackFunction_t callback,
                                      void* context) {
    UI_setButtonCallback(static_cast<button_t>(button),
                         reinterpret_cast<gpioCallbackFct_t>(callback), context);
}
