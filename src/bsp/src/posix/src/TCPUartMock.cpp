#include "TCPUartMock.h"
#include "BSPUtils.h"
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <cstring>
#include <freertos-utils/BaseTask.h>
#include <ros/ros.h>
#include <task.h>
#include <thread>

void TCPUartMock_listenTask(void* param) {
    auto* test = static_cast<TCPUartMock*>(param);
    test->waitForClient();
}

// FreeRTOS on Linux doesn't work very well with system calls. These threads prevent the kernel
// from hanging on the blocking socket calls.
void rxThread(TCPUartMock* context) {
    BSPUtils::blockSignals();

    while (true) {
        std::unique_lock<std::mutex> lock(context->m_recvMutex);
        context->m_startRecvSignal.wait(lock);

        context->m_recvRet = ::recv(context->m_clientFd.value(), context->m_recvBuffer,
                                    context->m_recvLength, MSG_WAITALL);
        context->m_recvEndedSignal.notify_one();
    }
}

void acceptThread(TCPUartMock* context) {
    BSPUtils::blockSignals();

    std::lock_guard<std::mutex> lock(context->m_acceptMutex);
    context->m_clientFd = ::accept(context->m_serverFd, (struct sockaddr*)&(context->m_address),
                                   (socklen_t*)&(context->m_addressLength));

    context->m_acceptEndedSignal.notify_one();
}

TCPUartMock::TCPUartMock(ILogger& logger) :
    m_logger(logger),
    m_listenTask("tcp_uart_mock_listen", tskIDLE_PRIORITY + 1, TCPUartMock_listenTask, this),
    m_port(0) {
    m_rxThread = std::thread(rxThread, this);
}

TCPUartMock::~TCPUartMock() { close(); }

void TCPUartMock::openSocket(int port) {
    if (port == 0) {
        m_logger.log(LogLevel::Info, "UART TCP mock port set to 0. Not initializing server.");
        return;
    }

    m_port = port;
    int serverFd;

    if ((serverFd = ::socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        m_logger.log(LogLevel::Error, "UART TCP mock socket creation failed");
        return;
    }
    m_serverFd = serverFd;

    m_address.sin_family = AF_INET;
    m_address.sin_addr.s_addr = INADDR_ANY;
    m_address.sin_port = htons(m_port);

    m_addressLength = sizeof(m_address);

    if (::bind(m_serverFd, (struct sockaddr*)&m_address, static_cast<socklen_t>(m_addressLength)) <
        0) {
        m_logger.log(LogLevel::Error, "UART TCP mock server binding failed");
        return;
    }

    if (m_listenTask.start()) {
        m_logger.log(LogLevel::Info, "UART TCP mock server waiting for client on port %d", m_port);
    } else {

        m_logger.log(LogLevel::Info, "UART TCP mock already listening on port %d", m_port);
    }
}

bool TCPUartMock::send(const uint8_t* buffer, uint16_t length) {
    if (!m_clientFd) {
        return false;
    }

    sigset_t mask = BSPUtils::blockSignals();
    ssize_t ret = ::send(m_clientFd.value(), buffer, length, 0);
    BSPUtils::unblockSignals(mask);

    return ret == length;
}

bool TCPUartMock::receive(uint8_t* buffer, uint16_t length) {
    if (!m_clientFd) {
        return false;
    }

    std::unique_lock<std::mutex> lock(m_recvMutex);

    m_recvBuffer = buffer;
    m_recvLength = length;

    m_startRecvSignal.notify_one();
    lock.unlock(); // Allow thread to do it's process
    m_recvEndedSignal.wait(lock);

    if (m_recvRet == 0) {
        // TODO: If we ever want to handle client reconnection in the simulation, do it here
        m_logger.log(LogLevel::Warn,
                     "Error while reading UART socket. Client has probably disconnected");
    }

    return m_recvRet == length;
}

bool TCPUartMock::isBusy() const { return false; }

void TCPUartMock::close() const {
    if (m_clientFd) {
        ::close(m_clientFd.value());
    }

    if (m_serverFd != 0) {
        ::close(m_serverFd);
    }
}

void TCPUartMock::waitForClient() {
    if (::listen(m_serverFd, 1) < 0) {
        m_logger.log(LogLevel::Error, "TCP UART mock server listen failed");
        return;
    }

    //    m_acceptThread = std::thread(acceptThread, this);
    //    std::unique_lock<std::mutex> lock(m_acceptMutex);
    //    m_acceptEndedSignal.wait(lock);

    if (m_clientFd < 0) {
        m_logger.log(LogLevel::Error, "TCP UART mock: Client acceptation failed");
    } else {
        m_logger.log(LogLevel::Info, "TCP UART mock: Client connected");
    }
}
