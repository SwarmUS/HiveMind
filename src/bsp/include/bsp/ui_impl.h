#ifndef __UI_IMPL_H_
#define __UI_IMPL_H_

#include "bsp/ui.h"

class UIImpl : public UI {
  public:
    UIImpl();
    ~UIImpl();

    int printf(const char* format, ...);
};

#endif // __UI_IMPL_H_
