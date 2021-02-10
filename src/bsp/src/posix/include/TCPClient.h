#ifndef __TCPCLIENT_H_
#define __TCPCLIENT_H_

#include "bsp/ITCPClient.h"
#include <condition_variable>
#include <logger/ILogger.h>
#include <mutex>
#include <netinet/in.h>
#include <optional>
#include <thread>

class TmpThread {
  public:
    std::thread m_rxThread;
    std::mutex m_recvMutex;
    std::condition_variable m_startRecvSignal;
    std::condition_variable m_recvEndedSignal;

    uint8_t* m_recvBuffer;
    uint16_t m_recvLength;
    int m_recvRet;
    int m_socketFd{};
};

class TCPClient : public ITCPClient {
  public:
    TCPClient(int socket, sockaddr_in address, ILogger& logger);

    ~TCPClient() override = default;

    bool receive(uint8_t* data, uint16_t length) override;

    bool send(const uint8_t* data, uint16_t length) override;

    bool close() override;

    friend void rxThread(TCPClient* context);

  private:
    ILogger& m_logger;
    const int m_socketFd{};
    const sockaddr_in m_address{};

    TmpThread* m_thread;
};

#endif // __TCPCLIENT_H_
