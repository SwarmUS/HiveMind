#ifndef __MOCK_UI_H_
#define __MOCK_UI_H_

#include <bsp/IUserInterface.h>
#include <cstdarg>
#include <gmock/gmock.h>

class UserInterfaceMock final : public IUserInterface {
  public:
    UserInterfaceMock(int& printCounter) : m_printCallCounter(printCounter) {}
    ~UserInterfaceMock() override = default;

    // GMock des not support variable arguments, so lets mock it ourselves
    int print(const char* format, ...) const override {
        va_list args;
        va_start(args, format);
        m_printCallCounter++;
        va_end(args);
        return 0;
    }

    int& m_printCallCounter;
};

#endif // __MOCK_UI_H_
