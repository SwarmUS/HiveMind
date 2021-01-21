#include "bsp/BSPContainer.h"
#include "BSP.h"
#include "HostUart.h"
#include "UserInterface.h"

IBSP& BSPContainer::getBSP() {
    static BSP s_bsp;

    return s_bsp;
}

IUserInterface& BSPContainer::getUserInterface() {
    static UserInterface s_ui;
    return s_ui;
}

IHostUart& BSPContainer::getHostUart() {
    static HostUart s_hostUart;
    return s_hostUart;
}