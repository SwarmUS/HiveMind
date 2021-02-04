#ifndef __TCPCLIENT_H_
#define __TCPCLIENT_H_

#include "bsp/ITCPClient.h"
#include <logger/ILogger.h>
#include <netinet/in.h>

class TCPClient : public ITCPClient {
  public:
    TCPClient(int socket, sockaddr_in address, ILogger& logger);

    ~TCPClient() override = default;

    int32_t receive(uint8_t* data, uint16_t length) override;

    int32_t send(const uint8_t* data, uint16_t length) override;

    bool close() override;

  private:
    ILogger& m_logger;
    const int m_socketFd;
    const sockaddr_in m_address;
};

#endif // __TCPCLIENT_H_
