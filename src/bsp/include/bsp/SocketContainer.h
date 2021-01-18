#ifndef __SOCKETCONTAINER_H_
#define __SOCKETCONTAINER_H_

#include "ITCPClient.h"
#include "TCPClientWrapper.h"
#include <logger/ILogger.h>
#include <optional>

/**@brief Containes the sockets for the application*/
namespace SocketContainer {
    /**
     * @brief Return an instance of the platform dependent BSP.
     *
     * @b Warning note that if you make multiple request, you will obtaine multiple TCPClientWrapper
     * objects, but under the hood, it's the same socket reference
     */
    std::optional<TCPClientWrapper> getHostClientSocket(const char* address,
                                                        int port,
                                                        const ILogger& logger);
} // namespace SocketContainer

#endif // __SOCKETCONTAINER_H_
