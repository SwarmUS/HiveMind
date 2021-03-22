#include "bsp/BSPContainer.h"
#include "BSP.h"
#include "HardwareCRC.h"
#include "SpiEsp.h"
#include "USB.h"
#include "UserInterface.h"
#include "logger/LoggerContainer.h"
#include <InterlocManager.h>

IBSP& BSPContainer::getBSP() {
    static BSP s_bsp;

    return s_bsp;
}

IUserInterface& BSPContainer::getUserInterface() {
    static UserInterface s_ui;
    return s_ui;
}

ICRC& BSPContainer::getCRC() {
    static HardwareCRC s_crc;

    return s_crc;
}

ISpiEsp& BSPContainer::getSpiEsp() {
    static SpiEsp s_spiEsp(getCRC(), LoggerContainer::getLogger());
    return s_spiEsp;
}

IUSB& BSPContainer::getUSB() {
    static USB s_usb(LoggerContainer::getLogger());
    return s_usb;
}

IInterlocManager& BSPContainer::getInterlocManager() {
    static InterlocManager s_interlocManager(LoggerContainer::getLogger());
    return s_interlocManager;
}
