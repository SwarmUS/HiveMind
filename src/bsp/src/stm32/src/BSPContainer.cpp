#include "bsp/BSPContainer.h"
#include "BSP.h"
#include "CRC.h"
#include "HostUart.h"
#include "UserInterface.h"

IBSP& BSPContainer::getBSP() {
    static BSP bsp;

    return bsp;
}

IUserInterface& BSPContainer::getUserInterface() {
    static UserInterface s_ui;
    return s_ui;
}

IHostUart& BSPContainer::getHostUart() {
    static HostUart s_hostUart(getCRC());

    return s_hostUart;
}

ICRC& BSPContainer::getCRC() {
    static CRC s_CRC;

    return s_CRC;
}