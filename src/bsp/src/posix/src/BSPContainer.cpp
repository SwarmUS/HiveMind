#include "bsp/BSPContainer.h"
#include "BSP.h"
#include "InterlocManager.h"
#include "SocketFactory.h"
#include "SpiEspMock.h"
#include "TCPClient.h"
#include "USBMock.h"
#include "UserInterface.h"
#include "bsp/SettingsContainer.h"
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

std::optional<std::reference_wrapper<ICommInterface>> BSPContainer::getHostCommInterface() {

    // TODO: handle closing socket on destruction
    static std::optional<TCPClient> s_clientSocket{};

    if (!s_clientSocket || !s_clientSocket.value().isConnected()) {

        ILogger& logger = LoggerContainer::getLogger();
        const uint32_t port = SettingsContainer::getHostPort();
        char address[MAX_IP_LENGTH];
        if (SettingsContainer::getHostIP(address, (uint8_t)MAX_IP_LENGTH) == 0) {
            logger.log(LogLevel::Error, "IP string too big for buffer");
            return {};
        }

        std::optional<TCPClient> socket = SocketFactory::createTCPClient(address, port, logger);
        if (socket) {
            s_clientSocket.emplace(socket.value());
        }
    }

    if (s_clientSocket) {
        return s_clientSocket.value();
    }

    return {};
}


std::optional<std::reference_wrapper<ICommInterface>> BSPContainer::getHostCommInterface() {

    // TODO: handle closing socket on destruction
    static std::optional<TCPClient> s_clientSocket{};

    if (!s_clientSocket || !s_clientSocket.value().isConnected()) {

        ILogger& logger = LoggerContainer::getLogger();
        const uint32_t port = SettingsContainer::getHostPort();
        char address[MAX_IP_LENGTH];
        if (SettingsContainer::getHostIP(address, (uint8_t)MAX_IP_LENGTH) == 0) {
            logger.log(LogLevel::Error, "IP string too big for buffer");
            return {};
        }

        std::optional<TCPClient> socket = SocketFactory::createTCPClient(address, port, logger);
        if (socket) {
            s_clientSocket.emplace(socket.value());
        }
    }

    if (s_clientSocket) {
        return s_clientSocket.value();
    }

    return {};
}