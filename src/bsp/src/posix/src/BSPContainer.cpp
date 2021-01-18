#include "bsp/BSPContainer.h"
#include "BSP.h"
#include "bsp/UserInterface.h"

IBSP& BSPContainer::getBSP() {
    static BSP s_bsp;

    return s_bsp;
}

IUserInterface& BSPContainer::getUserInterface() {
    static UserInterface s_ui;
    return s_ui;
}
