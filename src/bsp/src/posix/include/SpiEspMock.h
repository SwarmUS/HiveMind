#ifndef HIVE_CONNECT_SPISTM_H
#define HIVE_CONNECT_SPISTM_H

#include "bsp/ISpiEsp.h"
#include <BaseTask.h>
#include <condition_variable>
#include <logger/ILogger.h>
#include <mutex>
#include <netinet/in.h>
#include <optional>

/**
 * @brief mocks the SPI, on deconnection, will try to reconnect
 * */
class SpiEspMock : public ISpiEsp {
  public:
    SpiEspMock(ILogger& logger);
    ~SpiEspMock() override;

    void openSocket(int port);

    bool send(const uint8_t* buffer, uint16_t length) override;

    bool receive(uint8_t* buffer, uint16_t length) override;

    bool isBusy() const override;

    bool isConnected() const override;

    void close();
    friend void SpiMock_listenTask(void* param);

  private:
    ILogger& m_logger;

    BaseTask<configMINIMAL_STACK_SIZE * 2> m_listenTask;

    int m_serverFd{}, m_port;
    std::optional<int> m_clientFd;
    int m_addressLength{};
    struct sockaddr_in m_address {};

    void waitForClient();
};

#endif // HIVE_CONNECT_SPISTM_H