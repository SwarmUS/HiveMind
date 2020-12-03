#include "logger/logger.h"
#include <bsp/ui.h>

Logger::Logger(log_level_t level) { this->log_level = level; }

int Logger::log(log_level_t level, const char* format, ...) {
    if (level > this->log_level) {
        va_list args;
        va_start(args, format);
        int ret_value = UI.printf();
        va_end(args);

        return ret_value;
    }
}
