#include "bsp/BSPContainer.h"
#include "BSP.h"

IBSP& BSPContainer::getBSP() {
    static BSP s_bsp;

    return s_bsp;
}