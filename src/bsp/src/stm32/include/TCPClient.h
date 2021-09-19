#ifndef __TCPCLIENT_H_
#define __TCPCLIENT_H_

#include "bsp/ICommInterface.h"
#include <logger/ILogger.h>
#include <lwip/sockets.h>

class TCPClient : public ICommInterface {
  public:
    TCPClient(int socket, sockaddr_in address, ILogger& logger);

    ~TCPClient() override = default;

    bool receive(uint8_t* data, uint16_t length) override;

    bool send(const uint8_t* data, uint16_t length) override;

    bool isConnected() const override;

    ConnectionType getType() const override;

    bool close();

  private:
    ILogger& m_logger;
    bool m_connected;
    const int m_socketFd;
    const sockaddr_in m_address;
};

#endif // __TCPCLIENT_H_
