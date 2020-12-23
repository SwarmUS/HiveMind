#include "bsp/BSPFactory.h"
#include "BSP.h"

IBSP* BSPFactory::getBSP() {
    static BSP bsp;
    static bool isCreated = false;
    if (!isCreated) {
        new (&bsp) BSP();
    }

    return &bsp;
}