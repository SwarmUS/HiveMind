#ifndef __HOSTUART_H__
#define __HOSTUART_H__

#include "TCPServerMonitor.h"
#include "bsp/IHostUart.h"
#include <netinet/in.h>
#include <sys/socket.h>

class HostUart : public IHostUart {
  public:
    explicit HostUart(int port);
    ~HostUart() override;

    uint16_t receive(uint8_t* buffer, uint16_t length) const;
    bool send(const uint8_t* buffer, uint16_t length) override;
    bool isBusy() const override;

    friend void HostUart_listenTask(void* param);

  private:
    bool m_hasClient;
    int m_serverFd{}, m_clientFd{}, m_port;
    int m_addressLength{};
    struct sockaddr_in m_address {};

    void close() const;
    void waitForClient();
};

#endif //__HOSTUART_H__
