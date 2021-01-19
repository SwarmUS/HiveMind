#include "CRC.h"
#include "hal/hal.h"

CRC::CRC() {
    m_semaphore = xSemaphoreCreateBinary();

    xSemaphoreGive(m_semaphore);
}

CRC::~CRC() { vSemaphoreDelete(m_semaphore); }

uint32_t CRC::calculateCRC32(const void* data, uint32_t length) {
    uint32_t crc = 0;
    if (xSemaphoreTake(m_semaphore, (TickType_t)10) == pdTRUE) {
        crc = Hal_calculateCRC32((const uint8_t*)data, length);
        xSemaphoreGive(m_semaphore);
    }

    return crc;
}