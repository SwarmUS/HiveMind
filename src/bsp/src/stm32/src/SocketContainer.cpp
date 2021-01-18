#include "bsp/SocketContainer.h"

std::optional<TCPClientWrapper> SocketContainer::getHostClientSocket(const char* address,
                                                                     int port,
                                                                     const ILogger& logger) {
    (void)address;
    (void)port;
    (void)logger;

    return {};
}
