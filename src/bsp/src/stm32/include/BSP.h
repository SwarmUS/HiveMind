#ifndef __BSP_H_
#define __BSP_H_

#include "bsp/IBSP.h"
#include <BaseTask.h>
#include <FreeRTOSConfig.h>

class BSP : public IBSP {
  public:
    ~BSP() override;

    void initChip(void* args) override;

    uint16_t getUUId() const override;

    uint32_t generateRandomNumber() override;
};

#endif // __BSP_H_
