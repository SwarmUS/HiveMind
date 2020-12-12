#include "bsp/ui_impl.h"
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <hivemind_hal.h>

UIImpl::UIImpl() = default;
int UIImpl::print(const char* format, ...) {
    va_list args;
    va_start(args, format);
    int ret_value = printf(format, args);
    va_end(args);

    // Escape character to flush buffer
    printf("\r\n");

    return ret_value;
}
