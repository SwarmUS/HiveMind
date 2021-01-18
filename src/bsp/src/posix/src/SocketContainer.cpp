#include "bsp/SocketContainer.h"
#include "SocketFactory.h"
#include "TCPClient.h"

std::optional<TCPClientWrapper> SocketContainer::getHostClientSocket(const char* address,
                                                                     int port,
                                                                     const ILogger& logger) {

    static std::optional<TCPClient> s_clientSocket = {};
    static std::optional<TCPClientWrapper> s_wrapper = {};

    if (!s_clientSocket) {
        std::optional<TCPClient> socket = SocketFactory::createTCPClient(address, port, logger);

        if (socket) {
            s_clientSocket.emplace(socket.value());
            s_wrapper.emplace(s_clientSocket.value());
        }
    }

    return s_wrapper;
}
