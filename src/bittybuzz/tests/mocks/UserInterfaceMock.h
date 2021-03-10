#ifndef __USERINTERFACEMOCK_H_
#define __USERINTERFACEMOCK_H_

#include <bsp/IUserInterface.h>
#include <cstdarg>
#include <gmock/gmock.h>

class UserInterfaceMock final : public IUserInterface {
  public:
    int m_printCallCounter = 0;
    int m_flushCallCounter = 0;

    ~UserInterfaceMock() override = default;

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
};

#endif // __USERINTERFACEMOCK_H_
