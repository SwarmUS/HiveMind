#include "logger/logger_impl.h"
#include <bsp/ui.h>
#include <cstdarg>

LoggerImpl::LoggerImpl(LogLevel level, UI* ui) {
    m_log_level = level;
    m_ui = ui;
    m_semaphore = xSemaphoreCreateBinary();
}

LoggerImpl::~LoggerImpl() { vSemaphoreDelete(m_semaphore); }

LogRet LoggerImpl::log(LogLevel level, const char* format, ...) {
    if (xSemaphoreTake(m_semaphore, (TickType_t)10) == pdTRUE) {
        if (level >= m_log_level) {
            va_list args;
            va_start(args, format);
            int ret_value = m_ui->print(format, args);
            va_end(args);
            if (ret_value >= 0) {
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
