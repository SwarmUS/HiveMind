#include "bsp/SettingsContainer.h"
#include "BSP.h"
#include "bsp/BSPContainer.h"

std::shared_ptr<ros::NodeHandle> getRosNodeHandle() {
    BSP& bsp = static_cast<BSP&>(BSPContainer::getBSP());
    std::shared_ptr<ros::NodeHandle> handle = bsp.getRosNodeHandle();

    return handle;
}

uint16_t SettingsContainer::GetUUID() {
    auto handle = getRosNodeHandle();
    uint16_t uuid = handle->param("board_uuid", 1);

    return uuid;
}

uint32_t SettingsContainer::GetHostPort() {
    auto handle = getRosNodeHandle();
    uint32_t port = handle->param("host_port", 5555);

    return port;
}

std::string SettingsContainer::GetHostIP() {
    auto handle = getRosNodeHandle();
    std::string address = handle->param("host_tcp_address", std::string("127.0.0.1"));

    return address;
}