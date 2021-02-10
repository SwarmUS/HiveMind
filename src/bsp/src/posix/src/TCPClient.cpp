#include "TCPClient.h"
#include <BSPUtils.h>
#include <arpa/inet.h>
#include <cstdio>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

void rxThread(TmpThread* context) {
    BSPUtils::blockSignals();

    while (true) {
        std::unique_lock<std::mutex> lock(context->m_recvMutex);
        context->m_startRecvSignal.wait(lock);

        context->m_recvRet =
            ::recv(context->m_socketFd, context->m_recvBuffer, context->m_recvLength, MSG_WAITALL);
        context->m_recvEndedSignal.notify_one();
    }
}

TCPClient::TCPClient(int socket, sockaddr_in address, ILogger& logger) :
    m_logger(logger), m_socketFd(socket), m_address(address) {
    m_thread = new TmpThread();
    m_thread->m_socketFd = socket;
    m_thread->m_rxThread = std::thread(rxThread, this);
}

bool TCPClient::receive(uint8_t* data, uint16_t length) {
    std::unique_lock<std::mutex> lock(m_thread->m_recvMutex);

    m_thread->m_recvBuffer = data;
    m_thread->m_recvLength = length;

    m_thread->m_startRecvSignal.notify_one();
    lock.unlock(); // Allow thread to do it's process
    m_thread->m_recvEndedSignal.wait(lock);

    return m_thread->m_recvRet == length;
}

bool TCPClient::send(const uint8_t* data, uint16_t length) {
    sigset_t mask = BSPUtils::blockSignals();
    ssize_t ret = ::send(m_socketFd, data, length, 0);
    BSPUtils::unblockSignals(mask);

    return ret == length;
}

bool TCPClient::close() {
    m_logger.log(LogLevel::Info, "CLOSING SOCKET");
    int ret = ::close(m_socketFd);
    return ret == 0;
}
