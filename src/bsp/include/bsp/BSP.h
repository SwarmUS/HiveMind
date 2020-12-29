#ifndef __BSP_H_
#define __BSP_H_

#include "bsp/IBSP.h"

class BSP : public IBSP {
  public:
    BSP();
    ~BSP() override;

    void initChip() override;

    uint16_t getUUId() const override;
};

#endif // __BSP_H_