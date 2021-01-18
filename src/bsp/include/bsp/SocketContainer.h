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
     */
    std::optional<TCPClientWrapper> getHostClientSocket(const char* address,
                                                        int port,
                                                        const ILogger& logger);

} // namespace SocketContainer

#endif // __SOCKETCONTAINER_H_
