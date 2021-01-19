#ifndef __CRC_H__
#define __CRC_H__

#include "bsp/ICRC.h"
#include <FreeRTOS.h>
#include <semphr.h>

class CRC : public ICRC {
  public:
    CRC();
    ~CRC() override;

    uint32_t calculateCRC32(const void* data, uint32_t length) override;

  private:
    SemaphoreHandle_t m_semaphore;
};

#endif //__CRC_H__
