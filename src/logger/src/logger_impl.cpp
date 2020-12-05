#include "logger/logger_impl.h"
#include <bsp/ui.h>
#include <stdarg.h>

LoggerImpl::LoggerImpl(LogLevel level, UI* ui) {
    this->m_log_level = level;
    m_ui = ui;
}

LogRet LoggerImpl::log(LogLevel level, const char* format, ...) {
    if (level >= this->m_log_level) {
        va_list args;
        va_start(args, format);
        int ret_value = m_ui->printf(format, args);
        va_end(args);
        if (ret_value >= 0)
            return LogRet::Ok;
        else
            return LogRet::Error;
    }

    return LogRet::LowLevel;
}
