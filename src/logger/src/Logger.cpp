#include "logger/Logger.h"
#include <LockGuard.h>
#include <bsp/IUserInterface.h>
#include <cstdarg>

Logger::Logger(LogLevel level, const IUserInterface& ui) : m_ui(ui), m_mutex(10) {
    m_logLevel = level;
}

LogRet Logger::log(LogLevel level, const char* format, ...) {
    LockGuard lock = LockGuard(m_mutex);

    if (level >= m_logLevel) {
        va_list args;
        va_start(args, format);
        int retValue = m_ui.print(format, args);
        va_end(args);
        if (retValue >= 0) {
            return LogRet::Ok;
        }
        return LogRet::Error;
    }
    return LogRet::LowLevel;
}
