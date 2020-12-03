#ifndef __MOCK_UI_H_
#define __MOCK_UI_H_

#include "gmock/gmock.h"
#include <bsp/ui.h>

class UIMock : UI {
  public:
    UIMock();
    ~UIMock();

    MOCK_METHOD(int, printf, (const char* format, ...), (override));
};

#endif // __MOCK_UI_H_
