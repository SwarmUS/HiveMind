#ifndef __USERINTERFACE_H_
#define __USERINTERFACE_H_

#include "bsp/IUserInterface.h"

class UserInterface : public IUserInterface {
  public:
    UserInterface() = default;
    ~UserInterface() override = default;

    void flush() override;
    int print(const char* format, ...) override;
    int print(const char* format, va_list args) override;
    int printLine(const char* format, ...) override;
    int printLine(const char* format, va_list args) override;
};

#endif // __USERINTERFACE_H_
