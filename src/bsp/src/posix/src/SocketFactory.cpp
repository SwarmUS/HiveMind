#include "SocketFactory.h"
#include "TCPClient.h"
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

std::optional<TCPClient> SocketFactory::createTCPClient(const char* address,
                                                        uint32_t port,
                                                        ILogger& logger) {

    // Creating the socket
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
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
    if (connect(sockfd, (sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        logger.log(LogLevel::Info, "Could not connect to server %s : %d", address, port);
        ::close(sockfd);
        return {};
    }

    return TCPClient(sockfd, serverAddr, logger);
}
