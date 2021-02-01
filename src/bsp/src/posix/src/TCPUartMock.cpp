#include "TCPUartMock.h"
#include "BSPUtils.h"
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <cstring>
#include <ros/ros.h>
#include <task.h>

void TCPUartMock_listenTask(void* param) { auto* test = static_cast<TCPUartMock*>(param);
test->waitForClient(); }

TCPUartMock::TCPUartMock(ILogger& logger) : m_logger(logger), m_port(0) {}

TCPUartMock::~TCPUartMock() { close(); }

void TCPUartMock::openSocket(int port) {
    if (port == 0) {
        m_logger.log(LogLevel::Info, "UART TCP mock port set to 0. Not initializing server.");
        return;
    }

    m_port = port;
    int serverFd;

    if ((serverFd = ::socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        m_logger.log(LogLevel::Error, "UART TCP mock socket creation failed");
        return;
    }
    m_serverFd = serverFd;

    m_address.sin_family = AF_INET;
    m_address.sin_addr.s_addr = INADDR_ANY;
    m_address.sin_port = htons(m_port);

    m_addressLength = sizeof(m_address);

    if (::bind(m_serverFd, (struct sockaddr*)&m_address, static_cast<socklen_t>(m_addressLength)) <
        0) {
        m_logger.log(LogLevel::Error, "UART TCP mock server binding failed");
        return;
    }

    m_logger.log(LogLevel::Info, "UART TCP mock server waiting for client on port %d", m_port);

    const uint32_t stackSize = configMINIMAL_STACK_SIZE * 100;
    xTaskCreate(TCPUartMock_listenTask, "tcp_uart_mock_listen", stackSize,
                (void*)this, tskIDLE_PRIORITY + 1, NULL);
}

bool TCPUartMock::send(const uint8_t* buffer, uint16_t length) {
    if (!m_clientFd) {
        return false;
    }

    return ::send(m_clientFd.value(), buffer, length, 0) == (int)length;
}

int32_t TCPUartMock::receive(uint8_t* buffer, uint16_t length) const {
    if (!m_clientFd) {
        return -1;
    }

    int32_t ret = ::recv(m_clientFd.value(), buffer, length, 0);
    if (ret == 0) {
        // TODO: If we ever want to handle client reconnection in the simulation, do it here
        m_logger.log(LogLevel::Warn,
                     "Error while reading UART socket. Client has probably disconnected");
    }

    return ret;
}

bool TCPUartMock::isBusy() const { return false; }

void TCPUartMock::close() const {
    if (m_clientFd) {
        ::close(m_clientFd.value());
    }

    if (m_serverFd != 0) {
        ::close(m_serverFd);
    }
}

void TCPUartMock::waitForClient() {
    if (::listen(m_serverFd, 1) < 0) {
        m_logger.log(LogLevel::Error, "TCP UART mock server listen failed");
        return;
    }

    BSPUtils::blockSignals();
    m_clientFd = ::accept(m_serverFd, (struct sockaddr*)&m_address, (socklen_t*)&m_addressLength);
    BSPUtils::unblockSignals();

    if (m_clientFd < 0) {
        m_logger.log(LogLevel::Error, "TCP UART mock: Client acceptation failed");
    } else {
        m_logger.log(LogLevel::Info, "TCP UART mock: Client connected");
    }
}