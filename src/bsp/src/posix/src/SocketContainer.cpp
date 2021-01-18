#include "bsp/SocketContainer.h"
#include <functional>
#include "SocketFactory.h"
#include "TCPClient.h"
#include "bsp/TCPClientWrapper.h"

std::optional<std::reference_wrapper<TCPClientWrapper>> SocketContainer::getHostClientSocket(const char* address, int port, const ILogger& logger) {

    static std::optional<TCPClient> socket =  SocketFactory::createTCPClient(address, port, logger);
    if(socket){
        ITCPClient& test3 = socket.value();
        TCPClientWrapper test = TCPClientWrapper(test3);
        std::reference_wrapper<TCPClientWrapper> test2 = test;
        return test2;
    }



    return {};
}