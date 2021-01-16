#ifndef __SOCKETFACTORY_H_
#define __SOCKETFACTORY_H_

#include <logger/ILogger.h>
#include <optional>

class TCPClient;

namespace SocketFactory {

    std::optional<TCPClient> createTCPClient(const char* address, int port, const ILogger& logger);

} // namespace SocketFactory

#endif // __SOCKETFACTORY_H_
