#ifndef __BSP_H_
#define __BSP_H_

#include "PersistedStorageManager.h"
#include "bsp/IBSP.h"
#include <BaseTask.h>
#include <FreeRTOSConfig.h>

class BSP : public IBSP {
  public:
    BSP();
    ~BSP() override;

    void initChip(void* args) override;

    uint16_t getUUId() const override;

    uint32_t generateRandomNumber() override;

  private:
    PersistedStorageManager m_storage;
};

#endif // __BSP_H_
