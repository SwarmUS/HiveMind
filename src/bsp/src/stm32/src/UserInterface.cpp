#include "bsp/UserInterface.h"
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <hivemind_hal.h>

int UserInterface::print(const char* format, ...) const {
    va_list args;
    va_start(args, format);
    int retValue = printf(format, args);
    va_end(args);

    // Escape character to flush buffer
    printf("\r\n");

    return retValue;
}
