#include "TCPClient.h"
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

TCPClient::TCPClient(int socket, sockaddr_in address, ILogger& logger) :
    m_logger(logger), m_socketFd(socket), m_address(address), m_connected(true) {}

bool TCPClient::receive(uint8_t* data, uint16_t length) {
    ssize_t ret = ::recv(m_socketFd, data, length, MSG_WAITALL);
    m_connected = ret > 0;
    return ret == length;
}

bool TCPClient::send(const uint8_t* data, uint16_t length) {
    ssize_t ret = ::send(m_socketFd, data, length, 0) == length;
    m_connected = ret != -1;
    return ret;
}

bool TCPClient::isConnected() const { return m_connected; }

bool TCPClient::close() {
    m_logger.log(LogLevel::Info, "CLOSING SOCKET");
    m_connected = false;
    int ret = ::close(m_socketFd);
    return ret == 0;
}
