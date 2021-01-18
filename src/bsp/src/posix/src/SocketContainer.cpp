#include "bsp/SocketContainer.h"
#include "SocketFactory.h"
#include "TCPClient.h"
#include "bsp/TCPClientWrapper.h"
#include <functional>

std::optional<TCPClientWrapper> SocketContainer::getHostClientSocket(const char* address,
                                                                     int port,
                                                                     const ILogger& logger) {
    static std::optional<TCPClient> s_clientSocket = {};

    if (!s_clientSocket) {
        std::optional<TCPClient> socket = SocketFactory::createTCPClient(address, port, logger);

        if (socket) {
            s_clientSocket.emplace(socket.value());
            return TCPClientWrapper(s_clientSocket.value());
        }
    }

    return {};
}
