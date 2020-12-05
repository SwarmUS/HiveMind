#ifndef __UI_IMPL_H_
#define __UI_IMPL_H_

#include "bsp/ui.h"

class UIImpl : public UI {
  public:
    UIImpl();
    ~UIImpl() override{};

    int print(const char* format, ...) override;
};

#endif // __UI_IMPL_H_
