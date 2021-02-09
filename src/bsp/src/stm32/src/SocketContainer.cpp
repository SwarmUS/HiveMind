#include "bsp/SocketContainer.h"
#include "SocketFactory.h"
#include "TCPClient.h"
#include "bsp/SettingsContainer.h"
#include "logger/LoggerContainer.h"

std::optional<TCPClientWrapper> SocketContainer::getHostClientSocket() {

    static std::optional<TCPClient> s_clientSocket = {};
    static std::optional<TCPClientWrapper> s_wrapper = {};

    if (!s_clientSocket) {

        ILogger& logger = LoggerContainer::getLogger();
        const uint32_t port = SettingsContainer::getHostPort();
        char address[MAX_IP_LENGTH];
        if (SettingsContainer::getHostIP(address, MAX_IP_LENGTH) == 0) {
            logger.log(LogLevel::Error, "IP string too big for buffer");
            return s_wrapper;
        }

        std::optional<TCPClient> socket = SocketFactory::createTCPClient(address, port, logger);

        if (socket) {
            s_clientSocket.emplace(socket.value());
            s_wrapper.emplace(s_clientSocket.value());
        }
    }

    return s_wrapper;
}
