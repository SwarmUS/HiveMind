#include "bsp/BSPContainer.h"
#include "BSP.h"
#include "InterlocManager.h"
#include "SocketFactory.h"
#include "TCPClient.h"
#include "TCPServer.h"
#include "UserInterface.h"
#include "bsp/SettingsContainer.h"
#include "logger/LoggerContainer.h"
#include <interloc/InterlocContainer.h>
#include <mutex>

IBSP& BSPContainer::getBSP() {
    static BSP s_bsp;

    return s_bsp;
}

IUserInterface& BSPContainer::getUserInterface() {
    static UserInterface s_ui(getBSP());
    return s_ui;
}

IInterlocManager& BSPContainer::getInterlocManager() {
    static InterlocManager s_interlocManager(LoggerContainer::getLogger(),
                                             InterlocContainer::getInterlocUpdateInputQueue());
    return s_interlocManager;
}

std::optional<std::reference_wrapper<ICommInterface>> BSPContainer::getHostCommInterface() {
    static std::optional<TCPClient> s_clientSocket{};

    // TODO: Add logic for usb
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
        if (s_clientSocket) {
            s_clientSocket.value().close();
            s_clientSocket.reset();
        }

        // Create new socket
        std::optional<TCPClient> socket = SocketFactory::createTCPClient(address, port, logger);
        logger.log(LogLevel::Warn, "Trying to connect to host at %s : %d", address, port);
        if (socket) {
            s_clientSocket.emplace(socket.value());
        }
    }

    if (s_clientSocket) {
        return s_clientSocket.value();
    }
    return {};
}

std::optional<std::reference_wrapper<ICommInterface>> BSPContainer::getRemoteCommInterface() {
    static TCPServer s_remoteCommTCPServer(LoggerContainer::getLogger());
    static std::once_flag s_onceOpenSocket;

    std::call_once(s_onceOpenSocket, [&]() {
        std::shared_ptr<ros::NodeHandle> rosNodeHandle =
            static_cast<BSP&>(BSPContainer::getBSP()).getRosNodeHandle();
        int port = rosNodeHandle->param("remote_mock_port", 8001);
        s_remoteCommTCPServer.openSocket(port);
    });

    return s_remoteCommTCPServer;
}
