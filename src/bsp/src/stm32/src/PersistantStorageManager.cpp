#include "PersistantStorageManager.h"
#include <DefaultSettings.h>
#include <Task.h>
#include <cstdio>
#include <cstring>
#include <hal/hal_flash.h>
#include <interloc/InterlocBSPContainer.h>

#define DEFAULT_UUID 1

PersistantStorageManager::PersistantStorageManager(ILogger& logger) : m_logger(logger) {}

void PersistantStorageManager::loadFromFlash() {
    std::memcpy(&m_storage, reinterpret_cast<const void*>(USER_DATA_FLASH_START_ADDRESS),
                PersistedStorage::getSize());

    if (UUID_OVERRIDE != 0) {
        setUUID(UUID_OVERRIDE);
    }

    if (m_storage.m_uuid == UINT16_MAX) {
        m_logger.log(LogLevel::Error, "UUID is not set in FLASH. Use -D UUID_OVERRIDE to set it.");
        setUUID(DEFAULT_UUID);
    }

    InterlocBSPContainer::getAngleCalculator().setCalculatorParameters(
        m_storage.m_angleCalculatorParameters);
}

uint16_t PersistantStorageManager::getUUID() const { return m_storage.m_uuid; }

bool PersistantStorageManager::setUUID(uint16_t uuid) {
    m_storage.m_uuid = uuid;
    bool ret = saveToFlash();

    if (!ret) {
        m_logger.log(LogLevel::Error, "Error while saving UUID to flash");
    }

    return ret;
}

bool PersistantStorageManager::saveToFlash() {
    if (!Flash_eraseSector(USER_DATA_FLASH_SECTOR)) {
        return false;
    }

    // The size is given in words
    return Flash_program(USER_DATA_FLASH_START_ADDRESS, reinterpret_cast<uint8_t*>(&m_storage),
                         PersistedStorage::getSize());
}

void PersistantStorageManager::setAngleCalculatorParameters(
    const AngleCalculatorParameters& parameters) {
    m_storage.m_angleCalculatorParameters = parameters;
    InterlocBSPContainer::getAngleCalculator().setCalculatorParameters(parameters);

    bool ret = saveToFlash();

    if (!ret) {
        m_logger.log(LogLevel::Error, "Error while saving updating flash");
    }
}
