#include "BSP.h"
#include <hal/hal.h>

BSP::BSP() = default;
BSP::~BSP() = default;

void BSP::initChip(void* args) {
    (void)args;

    Hal_init();
}
