#include "TCPClient.h"
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

TCPClient::TCPClient(int socket, sockaddr_in address, const ILogger& logger) : m_logger(logger), m_socket(socket), m_address(address)
{}

// TCPClient::TCPClient(std::string address, int port, const ILogger& logger) : m_logger(logger) {

//     // Creating the socket
//     int sockfd = socket(AF_INET, SOCK_STREAM, 0);
//     if (sockfd < 0) {
//         m_logger.log(LogLevel::Error, "Could not open socket");
//     }

//     // Creating the server address
//     struct sockaddr_in serverAddr {};
//     serverAddr.sin_family = AF_INET;
//     serverAddr.sin_port = htons(port);
//     serverAddr.sin_addr.s_addr = inet_addr(address.c_str());

//     // Connect the socket
//     if (connect(sockfd, (sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
//         m_logger.log(LogLevel::Error, "Could not connect to server %s : %d", address.c_str(), port);
//     }
// }
