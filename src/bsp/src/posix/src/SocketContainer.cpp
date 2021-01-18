#include "bsp/SocketContainer.h"
#include "BSP.h"
#include "SocketFactory.h"
#include "TCPClient.h"
#include "bsp/BSPContainer.h"
#include <logger/LoggerContainer.h>

std::optional<TCPClientWrapper> SocketContainer::getHostClientSocket() {

    static std::optional<TCPClient> s_clientSocket = {};
    static std::optional<TCPClientWrapper> s_wrapper = {};

    if (!s_clientSocket) {

        BSP& bsp = static_cast<BSP&>(BSPContainer::getBSP());
        std::shared_ptr<ros::NodeHandle> handle = bsp.getRosNodeHandle();

        std::string address = handle->param("~host_address", std::string("127.0.0.1"));
        int port = handle->param("~host_port", 5555);
        ILogger& logger = LoggerContainer::getLogger();

        std::optional<TCPClient> socket =
            SocketFactory::createTCPClient(address.c_str(), (uint32_t)port, logger);

        if (socket) {
            s_clientSocket.emplace(socket.value());
            s_wrapper.emplace(s_clientSocket.value());
        }
    }

    return s_wrapper;
}
