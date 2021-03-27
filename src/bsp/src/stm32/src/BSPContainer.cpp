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

std::optional<std::reference_wrapper<ICommInterface>> BSPContainer::getHostCommInterface() {
    static std::optional<TCPClient> s_clientSocket{};

    // If disconnected, try to create a new socket
    if (!s_clientSocket || !s_clientSocket.value().isConnected()) {

        ILogger& logger = LoggerContainer::getLogger();
        const uint32_t port = SettingsContainer::getHostPort();
        char address[MAX_IP_LENGTH];
        if (SettingsContainer::getHostIP(address, (uint8_t)MAX_IP_LENGTH) == 0) {
            logger.log(LogLevel::Error, "IP string too big for buffer");
            return {};
        }

        // Close previous socket
        if(s_clientSocket){
            s_clientSocket.value().close();
        }  

        // Create new socket
        std::optional<TCPClient> socket = SocketFactory::createTCPClient(address, port, logger);
        if (socket) {
            s_clientSocket.emplace(socket.value());
        }
    }

    if(s_clientSocket){
        return s_clientSocket.value();
    }
    return {};
}


std::optional<std::reference_wrapper<ICommInterface>> BSPContainer::getRemoteCommInterface() {
    static SpiEsp s_spiEsp(getCRC(), LoggerContainer::getLogger());
    return s_spiEsp;
}
