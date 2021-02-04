#include "TCPClient.h"

TCPClient::TCPClient(int socket, sockaddr_in address, ILogger& logger) :
    m_logger(logger), m_socketFd(socket), m_address(address) {}

int32_t TCPClient::receive(uint8_t* data, uint16_t length) {
    return lwip_recv(m_socketFd, data, length, 0);
}

int32_t TCPClient::send(const uint8_t* data, uint16_t length) {
    return lwip_send(m_socketFd, data, length, 0);
}

bool TCPClient::close() {
    m_logger.log(LogLevel::Info, "CLOSING SOCKET");
    int ret = lwip_close(m_socketFd);
    return ret == 0;
}
