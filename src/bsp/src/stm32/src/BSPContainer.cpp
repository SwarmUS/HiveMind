#include "bsp/BSPContainer.h"
#include "BSP.h"
#include "CRC.h"
#include "HostUart.h"
#include "UserInterface.h"
#include "logger/LoggerContainer.h"

IBSP& BSPContainer::getBSP() {
    static BSP bsp;

    return bsp;
}

IUserInterface& BSPContainer::getUserInterface() {
    static UserInterface s_ui;
    return s_ui;
}

IHostUart& BSPContainer::getHostUart() {
    static TCPUartMock s_hostUart(getCRC(), LoggerContainer::getLogger());

    return s_hostUart;
}

ICRC& BSPContainer::getCRC() {
    static CRC s_CRC;

    return s_CRC;
}