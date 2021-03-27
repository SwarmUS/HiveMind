#include "TCPServer.h"
#include <BaseTask.h>
#include <Task.h>
#include <logger/Logger.h>
#include <ros/ros.h>

void TCPServer::listenTask(void* param) {
    auto* context = static_cast<TCPServer*>(param);
    context->waitForClient();
}

TCPServer::TCPServer(ILogger& logger) :
    m_logger(logger),
    m_connected(false),
    m_listenTask("tcp_spi_mock_listen", tskIDLE_PRIORITY + 1, TCPServer::listenTask, this),
    m_port(0) {}

TCPServer::~TCPServer() { close(); }

bool TCPServer::openSocket(int port) {
    if (port == 0) {
        m_logger.log(LogLevel::Info, "TCP server port set to 0. Not initializing server.");
        return false;
    }

    m_port = port;
    int serverFd;

    if ((serverFd = ::socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        m_logger.log(LogLevel::Error, "TCP server socket creation failed");
        return false;
    }
    m_serverFd = serverFd;

    m_address.sin_family = AF_INET;
    m_address.sin_addr.s_addr = INADDR_ANY;
    m_address.sin_port = htons(m_port);

    m_addressLength = sizeof(m_address);

    if (::bind(m_serverFd, (struct sockaddr*)&m_address, static_cast<socklen_t>(m_addressLength)) <
        0) {
        m_logger.log(LogLevel::Error, "TCP server server binding failed");
        return false;
    }

    if (::listen(m_serverFd, 1) < 0) {
        m_logger.log(LogLevel::Error, "TCP server server listen failed");
        return false;
    }

    if (m_listenTask.start()) {
        m_logger.log(LogLevel::Info, "TCP server server waiting for client on port %d", m_port);
    } else {

        m_logger.log(LogLevel::Info, "TCP server already listening on port %d", m_port);
    }
    return true;
}

bool TCPServer::send(const uint8_t* buffer, uint16_t length) {
    if (!m_clientFd) {
        return false;
    }
    ssize_t ret = ::send(m_clientFd.value(), buffer, length, 0);
    m_connected = ret != -1;

    return ret == length;
}

bool TCPServer::receive(uint8_t* buffer, uint16_t length) {
    if (!m_clientFd) {
        return false;
    }

    auto ret = ::recv(m_clientFd.value(), buffer, length, MSG_WAITALL);

    m_connected = ret > 0;
    return ret == length;
}

bool TCPServer::isConnected() const { return m_connected; }

void TCPServer::close() {
    m_connected = false;
    if (m_clientFd) {
        ::close(m_clientFd.value());
    }
    if (m_serverFd != 0) {
        ::close(m_serverFd);
        m_serverFd = -1;
    }
}

void TCPServer::waitForClient() {
    // Always tries to reconnect to client
    while (m_serverFd > 0) {
        if (!m_connected) {
            ::close(m_clientFd.value());
            m_clientFd =
                ::accept(m_serverFd, (struct sockaddr*)&m_address, (socklen_t*)&m_addressLength);

            if (m_clientFd < 0) {
                m_logger.log(LogLevel::Error, "TCP server: Client acceptation failed");
            } else {
                m_connected = true;
                m_logger.log(LogLevel::Info, "TCP server: Client connected");
            }
        }
        Task::delay(500);
    }
}
