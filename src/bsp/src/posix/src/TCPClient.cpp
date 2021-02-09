#include "TCPClient.h"
#include <BSPUtils.h>
#include <arpa/inet.h>
#include <cstdio>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

TCPClient::TCPClient(int socket, sockaddr_in address, ILogger& logger) :
    m_logger(logger), m_socketFd(socket), m_address(address) {}

bool TCPClient::receive(uint8_t* data, uint16_t length) {
    BSPUtils::blockSignals();
    ssize_t ret = ::recv(m_socketFd, data, length, MSG_WAITALL);
    BSPUtils::unblockSignals();

    return ret == length;
}

bool TCPClient::send(const uint8_t* data, uint16_t length) {
    BSPUtils::blockSignals();
    ssize_t ret = ::send(m_socketFd, data, length, 0);
    BSPUtils::unblockSignals();

    return ret == length;
}

bool TCPClient::close() {
    m_logger.log(LogLevel::Info, "CLOSING SOCKET");
    int ret = ::close(m_socketFd);
    return ret == 0;
}
