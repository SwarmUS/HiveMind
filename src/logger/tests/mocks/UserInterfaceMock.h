#ifndef __MOCK_UI_H_
#define __MOCK_UI_H_

#include <bsp/IUserInterface.h>
#include <cstdarg>
#include <gmock/gmock.h>

class UserInterfaceMock final : public IUserInterface {
  public:
    int& m_printCallCounter;
    Mutex m_mutex;

    UserInterfaceMock(int& printCounter) : m_printCallCounter(printCounter), m_mutex(10) {}
    ~UserInterfaceMock() override = default;

    Mutex& getPrintMutex() override { return m_mutex; }

    void flush() override{};

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
};

#endif // __MOCK_UI_H_
