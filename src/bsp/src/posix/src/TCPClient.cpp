#include "TCPClient.h"
#include <arpa/inet.h>
#include <cstdio>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

TCPClient::TCPClient(int socket, sockaddr_in address, const ILogger& logger) :
    m_logger(logger), m_socketFd(socket), m_address(address) {}

TCPClient::~TCPClient() {}

int32_t TCPClient::receive(uint8_t* data, uint16_t length) {
    return ::recv(m_socketFd, data, length, 0);
}

int32_t TCPClient::send(const uint8_t* data, uint16_t length) {
    return ::send(m_socketFd, data, length, 0);
}

bool TCPClient::close() {
    m_logger.log(LogLevel::Info, "CLOSING SOCKET");
    int ret = ::close(m_socketFd);
    if (ret == 0) {
        return true;
    }
    return false;
}
