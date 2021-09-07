#ifndef __USERINTERFACE_H_
#define __USERINTERFACE_H_

#include "bsp/IBSP.h"
#include "bsp/IUserInterface.h"
#include <array>
#include <ros/subscriber.h>
#include <string>

struct ButtonState {
    buttonCallbackFunction_t m_callback = NULL;
    void* m_context = NULL;
};

struct UIState {
    UIState();
    RgbColor m_rgbLed;
    std::array<bool, static_cast<int>(Button::BUTTON_MAX)> m_buttonStates;
    std::array<bool, static_cast<int>(LED::LED_MAX)> m_ledStates;
    uint8_t m_hexDisplay;
};

class UserInterface : public IUserInterface {
  public:
    UserInterface(const IBSP& bsp);
    ~UserInterface() override = default;

    Mutex& getPrintMutex() override;
    void flush() override;
    int print(const char* format, ...) override;
    int print(const char* format, va_list args) override;
    int printLine(const char* format, ...) override;
    int printLine(const char* format, va_list args) override;

    void setRGBLed(RgbColor color) override;
    void setLed(LED led, bool state) override;
    void setHexDisplay(uint8_t value) override;
    void setButtonCallback(Button button,
                           buttonCallbackFunction_t callback,
                           void* context) override;

  private:
    std::string uiStateToString();
    std::array<ros::Subscriber, static_cast<uint>(LED::LED_MAX)> m_buttonSubscribers;

    const IBSP& m_bsp;
    std::string m_accumulatedString;
    Mutex m_mutex;
    UIState m_uiState;
};

#endif // __USERINTERFACE_H_
