#include "BSP.h"
#include <hal/hal.h>

BSP::BSP() = default;
BSP::~BSP() = default;

void BSP::initChip(void* args) {
    (void)args;

    Hal_init();
}

uint16_t BSP::getUUId() const {
    // TODO: Change do the ID is obtained from persistent memory
    return 1;
}