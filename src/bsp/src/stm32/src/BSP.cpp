#include "BSP.h"
#include "bsp/SettingsContainer.h"
#include <hal/hal.h>

BSP::BSP() = default;
BSP::~BSP() = default;

void BSP::initChip(void* args) {
    (void)args;

    Hal_init();
}

uint16_t BSP::getUUId() const { return SettingsContainer::GetUUID(); }