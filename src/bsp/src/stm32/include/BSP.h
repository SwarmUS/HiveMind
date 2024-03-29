#ifndef __BSP_H_
#define __BSP_H_

#include "PersistantStorageManager.h"
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

    PersistantStorageManager& getStorage();

  private:
    PersistantStorageManager m_storage;

    static void buttonCallback(void* context);
    uint8_t m_currentRGBState = 0;
};

#endif // __BSP_H_
