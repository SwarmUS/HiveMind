#include "bsp/SocketContainer.h"
#include "SocketFactory.h"
#include "logger/LoggerContainer.h"
#include <TCPClient.h>

std::optional<TCPClientWrapper> SocketContainer::getHostClientSocket() {

    static std::optional<TCPClient> s_clientSocket = {};
    static std::optional<TCPClientWrapper> s_wrapper = {};

    if (!s_clientSocket) {

        ILogger& logger = LoggerContainer::getLogger();

        // TODO: Migrate settings to a settings manager that could provide a common interface
        // for settings between all builds. Would allow this class to be in bsp/common
        const char* hostAddress = "192.168.1.101";
        const uint32_t port = 5555;

        std::optional<TCPClient> socket = SocketFactory::createTCPClient(hostAddress, port, logger);

        if (socket) {
            s_clientSocket.emplace(socket.value());
            s_wrapper.emplace(s_clientSocket.value());
        }
    }

    return s_wrapper;
}
