#include "logger/Logger.h"
#include <bsp/IUserInterface.h>
#include <cstdarg>

Logger::Logger(LogLevel level, const IUserInterface& ui) : m_ui(ui) {
    m_logLevel = level;
    m_semaphore = xSemaphoreCreateBinary();

    if (m_semaphore == NULL) {
        m_ui.print("Error: Logger semaphore could not be created");
    }

    xSemaphoreGive(m_semaphore);
}

Logger::~Logger() { vSemaphoreDelete(m_semaphore); }

LogRet Logger::log(LogLevel level, const char* format, ...) const {
    if (xSemaphoreTake(m_semaphore, (TickType_t)10) == pdTRUE) {
        if (level >= m_logLevel) {
            va_list args;
            va_start(args, format);
            int retValue = m_ui.print(format, args);
            va_end(args);
            if (retValue >= 0) {
                xSemaphoreGive(m_semaphore);
                return LogRet::Ok;
            }
            xSemaphoreGive(m_semaphore);
            return LogRet::Error;
        }

        xSemaphoreGive(m_semaphore);
        return LogRet::LowLevel;
    }

    return LogRet::Error;
}
