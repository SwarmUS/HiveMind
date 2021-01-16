#ifndef __TCPCLIENT_H_
#define __TCPCLIENT_H_

#include "SocketFactory.h"
#include "bsp/ITCPClient.h"
#include <logger/ILogger.h>
#include <netinet/in.h>

class TCPClient : public ITCPClient {
  public:
    ~TCPClient() override;

    void receive(uint8_t* data, uint16_t length) override;

    void send(const uint8_t data, uint16_t length) override;

    bool close() override;

  private:
    TCPClient(int socket, sockaddr_in address, const ILogger& logger);

    const ILogger& m_logger;
    const int m_socket;
    const sockaddr_in m_address;

    friend std::optional<TCPClient> SocketFactory::createTCPClient(const char* address,
                                                                   int port) const;
};

#endif // __TCPCLIENT_H_
