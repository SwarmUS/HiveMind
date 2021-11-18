#include "SocketFactory.h"
#include "TCPClient.h"
#include <lwip/sockets.h>
#include <sys/types.h>

std::optional<TCPClient> SocketFactory::createTCPClient(const char* address,
                                                        uint32_t port,
                                                        ILogger& logger,
                                                        uint8_t receiveTimout) {

    // Creating the socket
    int sockfd = lwip_socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        logger.log(LogLevel::Error, "Could not open socket");
        return {};
    }

    if(receiveTimout > 0){
        struct timeval tv = {receiveTimout, 0};
        if(lwip_setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (struct timeval *)&tv, sizeof(struct timeval)) < 0){
            logger.log(LogLevel::Error, "Could not set timout on socket");
            lwip_close(sockfd);
            return {};
        }
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
