#include "logger/Logger.h"
#include <LockGuard.h>
#include <bsp/IUserInterface.h>
#include <cstdarg>

Logger::Logger(LogLevel level, IUserInterface& ui) : m_ui(ui) { m_logLevel = level; }

LogRet Logger::log(LogLevel level, const char* format, ...) {

    va_list args;
    va_start(args, format);
    LogRet retValue = log(level, format, args);
    va_end(args);
    return retValue;
}

LogRet Logger::log(LogLevel level, const char* format, va_list args) {

    if (level >= m_logLevel) {
        LockGuard lock = LockGuard(m_ui.getPrintMutex());
        m_ui.print("[%c] ", logLevelToString(level));
        int retValue = m_ui.printLine(format, args);
        if (retValue >= 0) {
            return LogRet::Ok;
        }
        return LogRet::Error;
    }
    return LogRet::LowLevel;
}

char Logger::logLevelToString(LogLevel logLevel) {
    switch (logLevel) {
    case LogLevel::Debug:
        return 'D';
    case LogLevel::Info:
        return 'I';
    case LogLevel::Warn:
        return 'W';
    case LogLevel::Error:
        return 'E';
    default:
        return 'U';
    }
}
