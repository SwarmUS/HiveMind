#include "SpiEspMock.h"
#include <BaseTask.h>
#include <Task.h>
#include <logger/Logger.h>
#include <ros/ros.h>

void SpiMock_listenTask(void* param) {
    auto* test = static_cast<SpiEspMock*>(param);
    test->waitForClient();
}

SpiEspMock::SpiEspMock(ILogger& logger) :
    m_logger(logger),
    m_listenTask("tcp_spi_mock_listen", tskIDLE_PRIORITY + 1, SpiMock_listenTask, this),
    m_port(0) {}

SpiEspMock::~SpiEspMock() { close(); }

void SpiEspMock::openSocket(int port) {
    if (port == 0) {
        m_logger.log(LogLevel::Info, "SPI TCP mock port set to 0. Not initializing server.");
        return;
    }

    m_port = port;
    int serverFd;

    if ((serverFd = ::socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        m_logger.log(LogLevel::Error, "SPI TCP mock socket creation failed");
        return;
    }
    m_serverFd = serverFd;

    m_address.sin_family = AF_INET;
    m_address.sin_addr.s_addr = INADDR_ANY;
    m_address.sin_port = htons(m_port);

    m_addressLength = sizeof(m_address);

    if (::bind(m_serverFd, (struct sockaddr*)&m_address, static_cast<socklen_t>(m_addressLength)) <
        0) {
        m_logger.log(LogLevel::Error, "SPI TCP mock server binding failed");
        return;
    }

    if (m_listenTask.start()) {
        m_logger.log(LogLevel::Info, "SPI TCP mock server waiting for client on port %d", m_port);
    } else {

        m_logger.log(LogLevel::Info, "SPI TCP mock already listening on port %d", m_port);
    }
}

bool SpiEspMock::send(const uint8_t* buffer, uint16_t length) {
    if (!m_clientFd) {
        return false;
    }

    return ::send(m_clientFd.value(), buffer, length, 0) == length;
}

bool SpiEspMock::receive(uint8_t* buffer, uint16_t length) {
    if (!m_clientFd) {
        return false;
    }

    auto ret = ::recv(m_clientFd.value(), buffer, length, MSG_WAITALL);

    if (ret <= 0) {
        m_logger.log(LogLevel::Warn, "Error while reading SPI socket. Client has probably "
                                     "disconnected. Attempting reconnection...");
        ::close(m_clientFd.value());
        m_clientFd = {};
        // Only return when connection has been restored.
        while (!m_clientFd) {
            waitForClient();
        }
        // Returning false since error occurred.
        return false;
    }

    return ret == length;
}

bool SpiEspMock::isBusy() const { return false; }

bool SpiEspMock::isConnected() const { return m_clientFd.has_value(); }

void SpiEspMock::close() const {
    if (m_clientFd) {
        ::close(m_clientFd.value());
    }

    if (m_serverFd != 0) {
        ::close(m_serverFd);
    }
}

void SpiEspMock::waitForClient() {
    if (::listen(m_serverFd, 1) < 0) {
        m_logger.log(LogLevel::Error, "TCP SPI mock server listen failed");
        return;
    }

    m_clientFd = ::accept(m_serverFd, (struct sockaddr*)&m_address, (socklen_t*)&m_addressLength);

    if (m_clientFd < 0) {
        m_logger.log(LogLevel::Error, "TCP SPI mock: Client acceptation failed");
    } else {
        m_logger.log(LogLevel::Info, "TCP SPI mock: Client connected");
    }
}
