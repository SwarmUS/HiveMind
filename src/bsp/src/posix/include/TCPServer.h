#ifndef __TCPSERVER_H__
#define __TCPSERVER_H__

#include "bsp/ICommInterface.h"
#include <BaseTask.h>
#include <condition_variable>
#include <logger/ILogger.h>
#include <mutex>
#include <netinet/in.h>
#include <optional>

class TCPServer : public ICommInterface {
  public:
    TCPServer(ILogger& logger);
    ~TCPServer() override;

    /**
     *@brief opens the socket on a certain port
     *@return true if the socket could be opened, binded and then listen to the port, false if not*/
    bool openSocket(int port);

    bool send(const uint8_t* buffer, uint16_t length) override;

    bool receive(uint8_t* buffer, uint16_t length) override;

    bool isConnected() const override;

    void close();

  private:
    ILogger& m_logger;
    bool m_connected;

    BaseTask<configMINIMAL_STACK_SIZE * 2> m_listenTask;

    int m_serverFd{}, m_port;
    std::optional<int> m_clientFd;
    int m_addressLength{};
    struct sockaddr_in m_address {};

    void waitForClient();
    static void listenTask(void* param);
};

#endif // __TCPSERVER_H__
