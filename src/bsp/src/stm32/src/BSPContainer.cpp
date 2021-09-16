#include "bsp/BSPContainer.h"
#include "BSP.h"
#include "HardwareCRC.h"
#include "SocketFactory.h"
#include "SpiEsp.h"
#include "TCPClient.h"
#include "USB.h"
#include "UserInterface.h"
#include "bsp/SettingsContainer.h"
#include "logger/LoggerContainer.h"
#include <interloc/InterlocBSPContainer.h>

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

IInterlocManager& BSPContainer::getInterlocManager() {
    return InterlocBSPContainer::getInterlocManager();
}

std::optional<std::reference_wrapper<ICommInterface>> BSPContainer::getHostCommInterface() {
    static USB usb(LoggerContainer::getLogger());
    return usb;

    //    static std::optional<TCPClient> s_clientSocket{};
    //
    //    // If disconnected, try to create a new socket
    //    if (!s_clientSocket || !s_clientSocket.value().isConnected()) {
    //
    //        ILogger& logger = LoggerContainer::getLogger();
    //        const uint32_t port = SettingsContainer::getHostPort();
    //        char address[MAX_IP_LENGTH];
    //        if (SettingsContainer::getHostIP(address, (uint8_t)MAX_IP_LENGTH) == 0) {
    //            logger.log(LogLevel::Error, "IP string too big for buffer");
    //            return {};
    //        }
    //
    //        // Close previous socket
    //        if (s_clientSocket) {
    //            s_clientSocket.value().close();
    //            s_clientSocket.reset();
    //        }
    //
    //        // Create new socket
    //        std::optional<TCPClient> socket = SocketFactory::createTCPClient(address, port,
    //        logger); if (socket) {
    //            s_clientSocket.emplace(socket.value());
    //        }
    //    }
    //
    //    if (s_clientSocket) {
    //        return s_clientSocket.value();
    //    }
    //    return {};
}

std::optional<std::reference_wrapper<ICommInterface>> BSPContainer::getRemoteCommInterface() {
    static SpiEsp s_spiEsp(getCRC(), LoggerContainer::getLogger());
    return s_spiEsp;
}
