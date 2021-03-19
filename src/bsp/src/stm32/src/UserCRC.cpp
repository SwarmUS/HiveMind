#include "UserCRC.h"
#include "hal/hal.h"
#include <LockGuard.h>
#include <c-common/software_crc.h>

UserCRC::UserCRC() : m_mutex(10) {}

uint32_t UserCRC::calculateCRC32(const void* data, uint32_t length) {
    uint32_t crc = 0;
    LockGuard lock(m_mutex);

    crc = Hal_calculateCRC32((const uint8_t*)data, length);
    return crc;
}

uint8_t UserCRC::calculateCRC8(const void* data, uint32_t length) {
    return calculateCRC8_software(data, length);
}
