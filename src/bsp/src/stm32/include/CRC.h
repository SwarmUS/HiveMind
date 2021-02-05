#ifndef __CRC_H__
#define __CRC_H__

#include "bsp/ICRC.h"
#include <FreeRTOS.h>
#include <freertos-utils/Mutex.h>
#include <semphr.h>

class CRC : public ICRC {
  public:
    CRC();
    ~CRC() override = default;

    uint32_t calculateCRC32(const void* data, uint32_t length) override;

  private:
    Mutex m_mutex;
};

#endif //__CRC_H__
