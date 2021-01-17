#include "TCPClientWrapper.h"

TCPClientWrapper::TCPClientWrapper(ITCPClient& client) :
    m_client(client){}

TCPClientWrapper::~TCPClientWrapper() { TCPClientWrapper::close(); }

int32_t TCPClient::receive(uint8_t* data, uint16_t length) {
    return m_client.receive(m_socketFd, data, length, 0);
}

int32_t TCPClient::send(const uint8_t* data, uint16_t length) {
    return m_client.send(m_socketFd, data, length, 0);
}

bool TCPClient::close() {
    return m_client.close();
}
