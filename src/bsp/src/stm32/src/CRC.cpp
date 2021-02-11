#include "CRC.h"
#include "hal/hal.h"
#include <LockGuard.h>

CRC::CRC() : m_mutex(10) {}

uint32_t CRC::calculateCRC32(const void* data, uint32_t length) {
    uint32_t crc = 0;
    LockGuard lock(m_mutex);

    crc = Hal_calculateCRC32((const uint8_t*)data, length);
    return crc;
}
