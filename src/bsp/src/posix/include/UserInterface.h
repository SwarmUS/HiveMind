#ifndef __USERINTERFACE_H_
#define __USERINTERFACE_H_

#include "bsp/IBSP.h"
#include "bsp/IUserInterface.h"
#include <string>

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

  private:
    const IBSP& m_bsp;
    std::string m_accumulatedString;
    Mutex m_mutex;
};

#endif // __USERINTERFACE_H_
