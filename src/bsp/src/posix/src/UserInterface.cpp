#include "bsp/UserInterface.h"
#include <cstdio>
#include <ros/console.h>

int UserInterface::print(const char* format, ...) const {
    va_list args;
    va_start(args, format);
    int retValue = print(format, args);
    va_end(args);

    return retValue;
}

int UserInterface::print(const char* format, va_list args) const {
    const int bufferSize = 1024;
    char buffer[bufferSize];

    int retValue = vsnprintf(buffer, bufferSize, format, args);

    ROS_INFO("%s", buffer);
    return retValue;
}