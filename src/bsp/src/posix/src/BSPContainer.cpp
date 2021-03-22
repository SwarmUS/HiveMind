#include "bsp/BSPContainer.h"
#include "BSP.h"
#include "InterlocManager.h"
#include "SpiEspMock.h"
#include "USBMock.h"
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

ISpiEsp& BSPContainer::getSpiEsp() {
    static SpiEspMock s_spiEsp(LoggerContainer::getLogger());
    return s_spiEsp;
}

IUSB& BSPContainer::getUSB() {
    static USBMock s_usb;
    return s_usb;
}

IInterlocManager& BSPContainer::getInterlocManager() {
    static InterlocManager s_interlocManager;
    return s_interlocManager;
}
