#include "UserInterface.h"
#include <cstdio>
#include <ros/console.h>

UserInterface::UserInterface() : m_mutex(10) {}

Mutex& UserInterface::getPrintMutex() { return m_mutex; }

void UserInterface::flush() {
    ROS_INFO("%s", m_accumulatedString.c_str());
    m_accumulatedString = "";
}

int UserInterface::print(const char* format, ...) {
    va_list args;
    va_start(args, format);
    int retValue = print(format, args);
    va_end(args);

    return retValue;
}

int UserInterface::print(const char* format, va_list args) {
    // Copy varargs
    va_list vaCopy;
    va_copy(vaCopy, args);
    const int requiredLength = std::vsnprintf(NULL, 0, format, vaCopy);
    va_end(vaCopy);

    // Create a string with adequate length
    std::string tmpStr;
    tmpStr.resize((size_t)requiredLength);

    // Build a new string
    int retValue = vsnprintf(tmpStr.data(), tmpStr.size() + 1, format, args);
    m_accumulatedString = m_accumulatedString + tmpStr;

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
