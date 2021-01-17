#include "comm/TCPDataStream.h"

TCPDataStream::TCPDataStream(ITCPClient& client) : m_client(client) {}

// Should I really close the client here or not?
TCPDataStream::~TCPDataStream() { m_client.close(); }

int32_t TCPDataStream::receive(uint8_t* data, uint16_t length) {
    return m_client.receive(data, length);
}

int32_t TCPDataStream::send(const uint8_t* data, uint16_t length) {
    return m_client.send(data, length);
}
