#include "UserInterface.h"
#include <cstdint>
#include <cstdio>
#include <hivemind_hal.h>

UserInterface::UserInterface() : m_mutex(10) {}

Mutex& UserInterface::getPrintMutex() { return m_mutex; }

void UserInterface::flush() {
    // Escape character to flush buffer
    printf("\r\n");
}
int UserInterface::print(const char* format, ...) {
    va_list args;
    va_start(args, format);
    int retValue = print(format, args);
    va_end(args);

    return retValue;
}

int UserInterface::print(const char* format, va_list args) {
    int retValue = vprintf(format, args);

    return retValue;
}

int UserInterface::printLine(const char* format, ...) {
    va_list args;
    va_start(args, format);
    int retValue = printLine(format, args);
    va_end(args);

    return retValue;
}

int UserInterface::printLine(const char* format, va_list args) {
    int retValue = print(format, args);
    flush();

    return retValue;
}
