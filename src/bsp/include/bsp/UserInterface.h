#ifndef __USERINTERFACE_H_
#define __USERINTERFACE_H_

#include "bsp/IUserInterface.h"

class UserInterface : public IUserInterface {
  public:
    UserInterface() = default;
    ~UserInterface() override = default;

    int print(const char* format, ...) const override;
    int print(const char* format, va_list args) const override;
};

#endif // __USERINTERFACE_H_
