#include "bsp/BSP.h"

#include <FreeRTOS.h>
#include <hal/hal.h>
#include <hivemind_hal.h>
#include <task.h>
#include <timers.h>

BSP::BSP() = default;
BSP::~BSP() = default;

void BSP::initChip() { Hal_init(); }

uint16_t BSP::getUUId() const {
    // TODO: Change do the ID is obtained from persistent memory
    return 1;
}
