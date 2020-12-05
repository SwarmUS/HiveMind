#ifndef __MOCK_UI_H_
#define __MOCK_UI_H_

#include <bsp/ui.h>
#include <gmock/gmock.h>
#include <stdarg.h>

class UIMock : public UI {
  public:
    ~UIMock() {}

    // GMock des not support variable arguments, so lets mock it ourselves
    int print(const char* format, ...) override {
        va_list args;
        va_start(args, format);
        print_called++;
        va_end(args);
        return 0;
    }

    int print_called = 0;
};

#endif // __MOCK_UI_H_
