#include "bsp/BSPContainer.h"
#include "BSP.h"
#include "UserInterface.h"

IBSP& BSPContainer::getBSP() {
    static BSP bsp;

    return bsp;
}

IUserInterface& BSPContainer::getUserInterface() {
    static UserInterface s_ui;
    return s_ui;
}
