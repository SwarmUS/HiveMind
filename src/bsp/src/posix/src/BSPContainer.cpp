#include "bsp/BSPContainer.h"
#include "BSP.h"

IBSP* BSPContainer::getBSP() {
    static BSP bsp;

    return &bsp;
}