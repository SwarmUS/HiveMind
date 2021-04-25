#include "BSP.h"
#include "bsp/SettingsContainer.h"
#include <hal/hal.h>
#include <logger/LoggerContainer.h>

BSP::BSP() : m_storage(LoggerContainer::getLogger()) {}
BSP::~BSP() = default;

void BSP::initChip(void* args) {
    (void)args;

    Hal_init();
    m_storage.loadFromFlash();
}

uint16_t BSP::getUUId() const { return m_storage.getUUID(); }

uint32_t BSP::generateRandomNumber() { return Hal_generateRandomNumber(); }
