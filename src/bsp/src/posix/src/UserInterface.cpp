#include "bsp/UserInterface.h"
#include <cstdarg>
#include <cstdio>
#include <ros/console.h>

int UserInterface::print(const char* format, ...) const {
    const int bufferSize = 1024;
    char buffer[bufferSize];
    int retValue = 0;

    va_list args;
    va_start(args, format);
    retValue = snprintf(buffer, bufferSize, format, args);
    va_end(args);

    ROS_INFO("%s", buffer);
    return retValue;
}
