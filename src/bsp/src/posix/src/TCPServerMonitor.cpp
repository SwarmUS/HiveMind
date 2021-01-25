#include "TCPServerMonitor.h"
#include <cstdlib>

TCPServerMonitor::TCPServerMonitor() {
    m_maxFd = 0;
    m_serverFd = 0;
    FD_ZERO(&m_fileDescriptorSet);
}

void TCPServerMonitor::addServer(int serverFd) {
    m_serverFd = serverFd;
    addConn(m_serverFd);
}

void TCPServerMonitor::addConn(int conn) {
    FD_SET(conn, &m_fileDescriptorSet);

    if (conn > m_maxFd) {
        m_maxFd = conn;
    }
}

void TCPServerMonitor::removeConn(int conn) {
    FD_CLR(conn, &m_fileDescriptorSet);
    m_maxFd = m_serverFd; // reset the max since we closed a socket.
}

bool TCPServerMonitor::waitIncoming(int conn) {
    // wait for activity on socket
    fd_set tempFileDescriptorSet = m_fileDescriptorSet;
    do {
        int sel = select(m_maxFd + 1, &tempFileDescriptorSet, NULL, NULL, NULL);
        if (sel < 0) {
            return false;
        }

    } while (!FD_ISSET(conn, &tempFileDescriptorSet));

    return true;
}
