#include "bsp/BSPContainer.h"
#include "BSP.h"
#include "SpiEspMock.h"
#include "TCPUartMock.h"
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

IHostUart& BSPContainer::getHostUart() {
    static TCPUartMock s_hostUart(LoggerContainer::getLogger());
    return s_hostUart;
}

ISpiEsp& BSPContainer::getSpiEsp() {
    static SpiEspMock s_spiEsp(LoggerContainer::getLogger());
    return s_spiEsp;
}

IUSB& BSPContainer::getUSB() {
    static USBMock s_usb();
    return s_usb;
}