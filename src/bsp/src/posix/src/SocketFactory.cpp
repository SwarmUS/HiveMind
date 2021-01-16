#include "SocketFactory.h"
#include "TCPClient.h"
#include <arpa/inet.h>
#include <logger/ILogger.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

SocketFactory::SocketFactory(const ILogger& logger) : m_logger(logger) {}

std::optional<TCPClient> SocketFactory::createTCPClient(const char* address, int port) const {

    // Creating the socket
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        m_logger.log(LogLevel::Error, "Could not open socket");
        return {};
    }

    // Creating the server address
    struct sockaddr_in serverAddr {};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = inet_addr(address);

    // Connect the socket
    if (connect(sockfd, (sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        m_logger.log(LogLevel::Error, "Could not connect to server %s : %d", address, port);
        return {};
    }

    return TCPClient(sockfd, serverAddr, m_logger);
}
