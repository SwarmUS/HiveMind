#include "UserInterface.h"
#include <cstdint>
#include <cstdio>
#include <hivemind_hal.h>

int UserInterface::print(const char* format, ...) const {
    va_list args;
    va_start(args, format);
    int retValue = print(format, args);
    va_end(args);

    return retValue;
}

int UserInterface::print(const char* format, va_list args) const {
    int retValue = vprintf(format, args);

    // Escape character to flush buffer
    printf("\r\n");

    return retValue;
}
