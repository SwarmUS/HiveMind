#ifndef __USERINTERFACE_H_
#define __USERINTERFACE_H_

#include "bsp/IUserInterface.h"

class UserInterface : public IUserInterface {
  public:
    UserInterface();
    ~UserInterface() override = default;

    Mutex& getPrintMutex() override;

    void flush() override;
    int print(const char* format, ...) override;
    int print(const char* format, va_list args) override;
    int printLine(const char* format, ...) override;
    int printLine(const char* format, va_list args) override;

    void setRGBLed(RgbColor color) override;
    void setLed(RgbColor color) override;
    void setHexDisplay(uint8_t value) override;
    void setButtonCallback(Button button,
                           buttonCallbackFunction_t callback,
                           void* context) override;

  private:
    Mutex m_mutex;
};

#endif // __USERINTERFACE_H_
