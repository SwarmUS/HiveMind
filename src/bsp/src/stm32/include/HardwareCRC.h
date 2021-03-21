#ifndef __HARDWARECRC_H__
#define __HARDWARECRC_H__

#include "bsp/ICRC.h"
#include <FreeRTOS.h>
#include <Mutex.h>
#include <semphr.h>

class HardwareCRC : public ICRC {
  public:
    HardwareCRC();
    ~HardwareCRC() override = default;

    uint32_t calculateCRC32(const void* data, uint32_t length) override;
    uint8_t calculateCRC8(const void* data, uint32_t length) override;

  private:
    Mutex m_mutex;
};

#endif //__CRC_H__
