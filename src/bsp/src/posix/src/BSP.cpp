#include "bsp/BSP.h"

BSP::BSP() = default;
BSP::~BSP() = default;

void BSP::initChip() {}

uint16_t BSP::getUUId() const {
    // TODO: Change do the ID is obtained from persistent memory or ROS command line argument
    return 1;
}