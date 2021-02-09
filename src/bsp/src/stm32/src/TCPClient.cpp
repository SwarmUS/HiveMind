#include "TCPClient.h"

TCPClient::TCPClient(int socket, sockaddr_in address, ILogger& logger) :
    m_logger(logger), m_socketFd(socket), m_address(address) {}

bool TCPClient::receive(uint8_t* data, uint16_t length) {
    ssize_t receivedBytes = 0;

    // MSG_WAITALL is not implemented in LwIP
    while (receivedBytes < length) {
        ssize_t recvSize = lwip_recv(m_socketFd, (data + receivedBytes),
                                     static_cast<size_t>(length - receivedBytes), 0);

        if (recvSize == -1) {
            return false;
        }

        receivedBytes += recvSize;
    }

    return true;
}

bool TCPClient::send(const uint8_t* data, uint16_t length) {
    return lwip_send(m_socketFd, data, length, 0) == length;
}

bool TCPClient::close() {
    m_logger.log(LogLevel::Info, "CLOSING SOCKET");
    int ret = lwip_close(m_socketFd);
    return ret == 0;
}
