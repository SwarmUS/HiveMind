#ifndef __USERINTERFACEMOCK_H_
#define __USERINTERFACEMOCK_H_

#include <bsp/IUserInterface.h>
#include <cstdarg>
#include <gmock/gmock.h>

class UserInterfaceMock : public IUserInterface {
  public:
    int m_printCallCounter = 0;
    int m_flushCallCounter = 0;
    Mutex m_mutex;

    UserInterfaceMock() : m_mutex(10) {}

    ~UserInterfaceMock() override = default;

    Mutex& getPrintMutex() override { return m_mutex; }

    void flush() override { m_flushCallCounter++; };

    int printLine(const char* format, ...) override {
        va_list args;
        va_start(args, format);
        m_printCallCounter++;
        va_end(args);
        return 0;
    }

    int printLine(const char* format, va_list args) override {
        (void)format;
        (void)args;
        m_printCallCounter++;
        return 0;
    }

    // GMock des not support variable arguments, so lets mock it ourselves
    int print(const char* format, ...) override {
        va_list args;
        va_start(args, format);
        m_printCallCounter++;
        va_end(args);
        return 0;
    }

    int print(const char* format, va_list args) override {
        (void)format;
        (void)args;
        m_printCallCounter++;
        return 0;
    }

    void setRGBLed(RgbColor color) override { (void)color; }

    void setLed(LED led, bool state) override {
        (void)led;
        (void)state;
    }

    void setHexDisplay(uint8_t value) override { (void)value; }

    void setButtonCallback(Button button,
                           buttonCallbackFunction_t callback,
                           void* context) override {
        (void)button;
        (void)callback;
        (void)context;
    }
};

#endif // __USERINTERFACEMOCK_H_
