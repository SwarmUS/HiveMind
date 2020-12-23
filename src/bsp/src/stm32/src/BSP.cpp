#include "BSP.h"

#include <FreeRTOS.h>
#include <hal/hal.h>
#include <hivemind_hal.h>
#include <task.h>
#include <timers.h>

BSP::BSP() = default;
BSP::~BSP() = default;

void BSP::initChip(int argc, char** argv) {
    (void)argc;
    (void)argv;

    Hal_init();
}
