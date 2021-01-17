#include "bsp/SocketContainer.h"
#include "SocketFactory.h"

std::optional<ITCPClient&> SocketContainer::getHostClientSocket(const char* address, int port, const ILogger& logger) {
    static  std::optional<ITCPClient> s_clientHost = {};

    if(!s_clientHost) {
        s_clientHost = SocketFactory::createTCPClient(address, port, logger);
    }

    if(s_clientHost){
        s_clientHost.value()
    }


    return {};
}