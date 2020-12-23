#include "BSP.h"
#include <hal/hal.h>

BSP::BSP() = default;
BSP::~BSP() = default;

void BSP::initChip(int argc, char** argv) {
    (void)argc;
    (void)argv;

    Hal_init();
}
