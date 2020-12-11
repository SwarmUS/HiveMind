#include "bsp/ui_impl.h"
#include <cstdarg>
#include <cstdio>

UIImpl::UIImpl() = default;

int UIImpl::print(const char* format, ...) {

    va_list args;
    va_start(args, format);
    int ret_value = printf(format, args);
    va_end(args);

    return ret_value;
}
