#ifndef __SOCKETFACTORY_H_
#define __SOCKETFACTORY_H_

#include <logger/ILogger.h>
#include <optional>

// Prevents circular dependency
class TCPClient;

class SocketFactory {
  public:
    SocketFactory(const ILogger& logger);
    ~SocketFactory() = default;

    std::optional<TCPClient> createTCPClient(const char* address, int port) const;

  private:
    const ILogger& m_logger;
};

#endif // __SOCKETFACTORY_H_
