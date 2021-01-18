#include "bsp/SocketContainer.h"
#include "BSP.h"
#include "SocketFactory.h"
#include "TCPClient.h"
#include "bsp/BSPContainer.h"
#include "logger/LoggerContainer.h"

std::optional<TCPClientWrapper> SocketContainer::getHostClientSocket() {

    static std::optional<TCPClient> s_clientSocket = {};
    static std::optional<TCPClientWrapper> s_wrapper = {};

    if (!s_clientSocket) {
        std::string address;
        int port;

        BSP& bsp = static_cast<BSP&>(BSPContainer::getBSP());
        std::shared_ptr<ros::NodeHandle> handle = bsp.getRosNodeHandle();

        handle->getParam("host_address", address);
        handle->getParam("host_port", port);

        std::optional<TCPClient> socket =
            SocketFactory::createTCPClient(address.c_str(), (uint32_t)port, logger);

        if (socket) {
            s_clientSocket.emplace(socket.value());
            s_wrapper.emplace(s_clientSocket.value());
        }
    }

    return s_wrapper;
}
