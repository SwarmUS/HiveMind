#ifndef __TCPUARTMOCK_H__
#define __TCPUARTMOCK_H__

#include "bsp/IHostUart.h"
#include <BaseTask.h>
#include <condition_variable>
#include <logger/ILogger.h>
#include <mutex>
#include <netinet/in.h>
#include <optional>

class TCPUartMock : public IHostUart {
  public:
    TCPUartMock(ILogger& logger);
    ~TCPUartMock() override;

    void openSocket(int port);

    bool receive(uint8_t* buffer, uint16_t length) override;
    bool send(const uint8_t* buffer, uint16_t length) override;
    bool isBusy() const override;
    void close() const;

    bool isConnected() const override { return m_clientFd.has_value(); }

    friend void TCPUartMock_listenTask(void* param);

  private:
    ILogger& m_logger;

    BaseTask<configMINIMAL_STACK_SIZE * 2> m_listenTask;

    int m_serverFd{}, m_port;
    std::optional<int> m_clientFd;
    int m_addressLength{};
    struct sockaddr_in m_address {};

    void waitForClient();
};

#endif //__TCPUARTMOCK_H__
