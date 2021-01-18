#include "bsp/SocketContainer.h"

std::optional<TCPClientWrapper> SocketContainer::getHostClientSocket(const char* address,
                                                                     uint32_t port,
                                                                     const ILogger& logger) {
    (void)address;
    (void)port;
    (void)logger;

    return {};
}
