#include "PersistedStorageManager.h"
#include <DefaultSettings.h>
#include <Task.h>
#include <cstdio>
#include <cstring>
#include <hal/hal_flash.h>

#define DEFAULT_UUID 1

PersistedStorageManager::PersistedStorageManager(ILogger& logger) : m_logger(logger) {}

void PersistedStorageManager::loadFromFlash() {
    std::memcpy(&m_storage, reinterpret_cast<const void*>(USER_DATA_FLASH_START_ADDRESS),
                sizeof(m_storage));

    if (UUID_OVERRIDE != UINT16_MAX) {
        setUUID(UUID_OVERRIDE);
    }

    if (m_storage.m_uuid == UINT16_MAX) {
        m_logger.log(LogLevel::Error, "UUID is not set in FLASH. Use -D UUID_OVERRIDE to set it.");
        setUUID(DEFAULT_UUID);
    }
}

uint16_t PersistedStorageManager::getUUID() const { return m_storage.m_uuid; }

bool PersistedStorageManager::setUUID(uint16_t uuid) {
    m_storage.m_uuid = uuid;
    bool ret = saveToFlash();

    if (!ret) {
        m_logger.log(LogLevel::Error, "Error while saving UUID to flash");
    }

    return ret;
}

bool PersistedStorageManager::saveToFlash() {
    if (!Flash_eraseSector(USER_DATA_FLASH_SECTOR)) {
        return false;
    }

    return Flash_program(USER_DATA_FLASH_START_ADDRESS, reinterpret_cast<uint8_t*>(&m_storage),
                         sizeof(m_storage));
}
