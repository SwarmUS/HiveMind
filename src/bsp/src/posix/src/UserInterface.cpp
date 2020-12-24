#include "bsp/UserInterface.h"
#include <cstdarg>
#include <cstdio>

int UserInterface::print(const char* format, ...) const {

    va_list args;
    va_start(args, format);
    int retValue = printf(format, args);
    printf("\n");
    va_end(args);

    return retValue;
}
