#include "bsp/UserInterface.h"
#include <cstdarg>
#include <cstdio>
#include <ros/console.h>

int UserInterface::print(const char* format, ...) const {
    int retValue = 0;

    // ROS logging methods do not return the standard printf value. So let's just return -1 if there was an error
    try {
        va_list args;
        va_start(args, format);
        ROS_INFO(format, args);
        va_end(args);
    } catch (const ros::Exception& e) {
        retValue = -1;
    }

    return retValue;
}
