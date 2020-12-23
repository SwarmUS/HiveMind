#include "bsp/BSPFactory.h"
#include "BSP.h"

IBSP* BSPFactory::getBSP() {
    static BSP bsp;

    return &bsp;
}