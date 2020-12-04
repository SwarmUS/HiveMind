#ifndef __MOCK_UI_H_
#define __MOCK_UI_H_

#include <bsp/ui.h>
#include <gmock/gmock.h>

class UIMock : public UI {
  public:
    UIMock();
    ~UIMock();

    MOCK_METHOD(int, printf, (const char* format, ...), override);
};

#endif // __MOCK_UI_H_
