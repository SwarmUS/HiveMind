#include "bsp/SocketContainer.h"
#include "SocketFactory.h"
#include "bsp/TCPClientWrapper.h"

std::optional<std::reference_wrapper<TCPClientWrapper>> SocketContainer::getHostClientSocket(const char* address, int port, const ILogger& logger) {

    static std::optional<TCPClient> socket =  SocketFactory::createTCPClient(address, port, logger);
    if(socket){
        std::reference_wrapper<TCPClientWrapper> test = socket.value();
        return test;
    }



    return {};
}