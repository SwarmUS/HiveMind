#include "bsp/BSPContainer.h"
#include "BSP.h"
#include "CRC.h"
#include "HostUart.h"
#include "SpiEsp.h"
#include "UserInterface.h"
#include "logger/LoggerContainer.h"

IBSP& BSPContainer::getBSP() {
    static BSP s_bsp;

    return s_bsp;
}

IUserInterface& BSPContainer::getUserInterface() {
    static UserInterface s_ui;
    return s_ui;
}

IHostUart& BSPContainer::getHostUart() {
    static HostUart s_hostUart(getCRC(), LoggerContainer::getLogger());

    return s_hostUart;
}

ICRC& BSPContainer::getCRC() {
    static CRC s_crc;

    return s_crc;
}

ISpiEsp& BSPContainer::getSpiEsp() {
    static SpiEsp s_spiEsp(getCRC(), LoggerContainer::getLogger());
    return s_spiEsp;
}
