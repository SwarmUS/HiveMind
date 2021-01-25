#include "HostUart.h"
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <cstdio>
#include <cstring>
#include <ros/ros.h>
#include <task.h>

void HostUart_listenTask(void* param) { static_cast<HostUart*>(param)->waitForClient(); }

HostUart::HostUart(int port) {
    m_port = port;
    m_hasClient = false;

    int serverFd = 0;

    if ((serverFd = ::socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        ROS_ERROR("TCP server socket creation failed");
    }
    m_serverFd = serverFd;

    ROS_INFO("server FD: %d", m_serverFd);

    m_address.sin_family = AF_INET;
    m_address.sin_addr.s_addr = INADDR_ANY;
    m_address.sin_port = htons(m_port);

    m_addressLength = sizeof(m_address);

    if (::bind(m_serverFd, (struct sockaddr*)&m_address, static_cast<socklen_t>(m_addressLength)) <
        0) {
        ROS_ERROR("TCP server binding failed");
    }

    xTaskCreate(HostUart_listenTask, "host_uart_listen", configMINIMAL_STACK_SIZE * 10, (void*)this,
                tskIDLE_PRIORITY + 1, NULL);
}

HostUart::~HostUart() {
    close();
    ::close(m_serverFd);
}

bool HostUart::send(const uint8_t* buffer, uint16_t length) {
    if (!m_hasClient) {
        return false;
    }

    return ::send(m_clientFd, buffer, length, 0) >= 0;
}

uint16_t HostUart::receive(uint8_t* buffer, uint16_t length) const {
    if (!m_hasClient) {
        return 0;
    }

    uint16_t ret = ::recv(m_clientFd, buffer, length, 0);
    if (ret == 0) {
        ROS_ERROR("Error while reading UART socket");
    }

    return ret;
}

bool HostUart::isBusy() const { return false; }

void HostUart::close() const { ::close(m_clientFd); }

void HostUart::waitForClient() {
    if (::listen(m_serverFd, 1) < 0) {
        ROS_ERROR("TCP server listen failed");
    }

    m_clientFd = ::accept(m_serverFd, (struct sockaddr*)&m_address, (socklen_t*)&m_addressLength);

    if (m_clientFd < 0) {
        ROS_ERROR("TCP server accept failed");
    } else {
        ROS_INFO("TCP client connected");
        m_hasClient = true;
    }
}