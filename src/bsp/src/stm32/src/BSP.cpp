#include "BSP.h"
#include "bsp/SettingsContainer.h"
#include <hal/hal.h>

BSP::~BSP() = default;

void BSP::initChip(void* args) {
    (void)args;

    Hal_init();
}

uint16_t BSP::getUUId() const { return SettingsContainer::getUUID(); }

uint32_t BSP::generateRandomNumber() { return static_cast<uint32_t>(rand()); }
