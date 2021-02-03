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
        std::string hostAddress = SettingsContainer::GetHostIP();
        const uint32_t port = SettingsContainer::GetHostPort();

        std::optional<TCPClient> socket =
            SocketFactory::createTCPClient(hostAddress.c_str(), port, logger);

        if (socket) {
            s_clientSocket.emplace(socket.value());
            s_wrapper.emplace(s_clientSocket.value());
        }
    }

    return s_wrapper;
}
