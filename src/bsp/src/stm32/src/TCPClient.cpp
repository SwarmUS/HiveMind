#include "TCPClient.h"

TCPClient::TCPClient(int socket, sockaddr_in address, ILogger& logger) :
    m_logger(logger), m_connected(socket >= 0), m_socketFd(socket), m_address(address) {}

bool TCPClient::receive(uint8_t* data, uint16_t length) {
    ssize_t receivedBytes = 0;

    // MSG_WAITALL is not implemented in LwIP
    while (receivedBytes < length) {
        ssize_t recvSize = lwip_recv(m_socketFd, (data + receivedBytes),
                                     static_cast<size_t>(length - receivedBytes), 0);

        if (recvSize <= 0) {
            m_connected = false;
            return false;
        }

        receivedBytes += recvSize;
    }

    return true;
}

bool TCPClient::send(const uint8_t* data, uint16_t length) {
    ssize_t ret = lwip_send(m_socketFd, data, length, 0);
    m_connected = ret != -1;
    return ret == length;
}

bool TCPClient::isConnected() const { return m_connected; }

bool TCPClient::close() {
    m_logger.log(LogLevel::Info, "Closing Socket");
    m_connected = false;
    int ret = lwip_close(m_socketFd);
    return ret == 0;
}
