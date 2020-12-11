#include "bsp/ui_impl.h"
#include <stdarg.h>
#include <stdio.h>
#include <hivemind_hal.h>

UIImpl::UIImpl() {}
int UIImpl::print(const char* format, ...) {
    va_list args;
    va_start(args, format);

    int string_size = vsnprintf(NULL, 0, format, args)+1;
    char string [string_size];
    va_end(args); //You can't use a va_list twice without reinitializing it

    va_start(args, format);
    vsnprintf(string, string_size, format, args);
    va_end(args);

    int test= 6;

    HAL_UART_Transmit(huart_print, (uint8_t*) string, string_size, HAL_MAX_DELAY);

    return 0;
}
