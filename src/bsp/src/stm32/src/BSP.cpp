#include "bsp/BSP.h"

#include <FreeRTOS.h>
#include <hal/hal.h>
#include <hivemind_hal.h>
#include <task.h>
#include <timers.h>

BSP::BSP() = default;
BSP::~BSP() = default;

void BSP::initChip() { Hal_init(); }
