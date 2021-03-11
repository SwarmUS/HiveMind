#include "logger/Logger.h"
#include <LockGuard.h>
#include <bsp/IUserInterface.h>
#include <cstdarg>

Logger::Logger(LogLevel level, IUserInterface& ui) : m_ui(ui) { m_logLevel = level; }

LogRet Logger::log(LogLevel level, const char* format, ...) {
    if (level >= m_logLevel) {
        va_list args;
        va_start(args, format);
        LockGuard lock = LockGuard(m_ui.getPrintMutex());
        int retValue = m_ui.printLine(format, args);
        va_end(args);
        if (retValue >= 0) {
            return LogRet::Ok;
        }
        return LogRet::Error;
    }
    return LogRet::LowLevel;
}
