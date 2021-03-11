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

  private:
    Mutex m_mutex;
};

#endif // __USERINTERFACE_H_
