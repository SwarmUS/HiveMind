#include "bsp/ui_impl.h"
#include <stdarg.h>

int UIImpl::printf(const char* format, ...) {

    va_list args;
    va_start(args, format);
    int ret_value = printf(format, args);
    va_end(args);

    return ret_value;
};
