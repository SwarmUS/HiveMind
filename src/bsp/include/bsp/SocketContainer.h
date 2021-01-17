#ifndef __SOCKETCONTAINER_H_
#define __SOCKETCONTAINER_H_

#include "ITCPClient.h"
#include <logger/ILogger.h>
#include <optional>

/**@brief Containes the sockets for the application*/
namespace SocketContainer {
    /**
     * @brief Return an instance of the platform dependent BSP.
     */
    std::optional<std::reference_wrapper<TCPClientWrapper>>  getHostClientSocket(const char* address, int port, const ILogger& logger);

} // namespace TCPClientHostContainer

#endif // __SOCKETCONTAINER_H_
