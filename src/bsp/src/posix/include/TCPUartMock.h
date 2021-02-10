#ifndef __TCPUARTMOCK_H__
#define __TCPUARTMOCK_H__

#include "bsp/IHostUart.h"
#include <condition_variable>
#include <freertos-utils/BaseTask.h>
#include <logger/ILogger.h>
#include <mutex>
#include <netinet/in.h>
#include <optional>
#include <sys/socket.h>
#include <thread>

class TCPUartMock : public IHostUart {
  public:
    TCPUartMock(ILogger& logger);
    ~TCPUartMock() override;

    void openSocket(int port);

    bool receive(uint8_t* buffer, uint16_t length) override;
    bool send(const uint8_t* buffer, uint16_t length) override;
    bool isBusy() const override;
    void close() const;

    friend void TCPUartMock_listenTask(void* param);
    friend void rxThread(TCPUartMock* context);
    friend void acceptThread(TCPUartMock* context);

  private:
    ILogger& m_logger;

    BaseTask<configMINIMAL_STACK_SIZE * 100> m_listenTask;

    int m_serverFd{}, m_port;
    std::optional<int> m_clientFd;
    int m_addressLength{};
    struct sockaddr_in m_address {};

    std::thread m_rxThread;
    std::mutex m_recvMutex;
    std::condition_variable m_startRecvSignal;
    std::condition_variable m_recvEndedSignal;

    std::thread m_acceptThread;
    std::mutex m_acceptMutex;
    std::condition_variable m_acceptEndedSignal;

    uint8_t* m_recvBuffer;
    uint16_t m_recvLength;
    ssize_t m_recvRet;

    void waitForClient();
};

#endif //__TCPUARTMOCK_H__
