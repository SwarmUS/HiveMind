#ifndef __SOCKETFACTORY_H_
#define __SOCKETFACTORY_H_

#include <cstdint>
#include <logger/ILogger.h>
#include <optional>

class TCPClient;

namespace SocketFactory {

    std::optional<TCPClient> createTCPClient(const char* address, uint32_t port, ILogger& logger);

} // namespace SocketFactory

#endif // __SOCKETFACTORY_H_
