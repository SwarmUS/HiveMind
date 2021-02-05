#include "SocketFactory.h"
#include "TCPClient.h"
#include <lwip/sockets.h>
#include <sys/types.h>

std::optional<TCPClient> SocketFactory::createTCPClient(const char* address,
                                                        uint32_t port,
                                                        ILogger& logger) {

    // Creating the socket
    int sockfd = lwip_socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        logger.log(LogLevel::Error, "Could not open socket");
        return {};
    }

    // Creating the server address
    struct sockaddr_in serverAddr {};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = inet_addr(address);

    // Connect the socket
    if (lwip_connect(sockfd, (sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        logger.log(LogLevel::Info, "Could not connect to server %s : %d", address, port);
        lwip_close(sockfd);
        return {};
    }

    return TCPClient(sockfd, serverAddr, logger);
}
