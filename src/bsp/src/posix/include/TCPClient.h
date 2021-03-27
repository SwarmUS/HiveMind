#ifndef __TCPCLIENT_H_
#define __TCPCLIENT_H_

#include "bsp/ICommInterface.h"
#include <condition_variable>
#include <logger/ILogger.h>
#include <mutex>
#include <netinet/in.h>
#include <optional>
#include <thread>

class TCPClient : public ICommInterface {
  public:
    TCPClient(int socket, sockaddr_in address, ILogger& logger);

    ~TCPClient() override = default;

    bool receive(uint8_t* data, uint16_t length) override;

    bool send(const uint8_t* data, uint16_t length) override;

    bool isConnected() const override;

    bool close();

    friend void rxThread(TCPClient* context);

  private:
    ILogger& m_logger;
    const int m_socketFd{};
    const sockaddr_in m_address{};
    bool m_connected;
};

#endif // __TCPCLIENT_H_
