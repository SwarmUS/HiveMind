#include "bsp/SettingsContainer.h"
#include "BSP.h"
#include "bsp/BSPContainer.h"

std::shared_ptr<ros::NodeHandle> getRosNodeHandle() {
    BSP& bsp = static_cast<BSP&>(BSPContainer::getBSP());
    std::shared_ptr<ros::NodeHandle> handle = bsp.getRosNodeHandle();

    return handle;
}

uint16_t SettingsContainer::getUUID() {
    auto handle = getRosNodeHandle();
    uint16_t uuid = handle->param("board_uuid", 1);

    return uuid;
}

uint32_t SettingsContainer::getHostPort() {
    auto handle = getRosNodeHandle();
    uint32_t port = (uint32_t)handle->param("host_tcp_port", 55551);

    return port;
}

uint8_t SettingsContainer::getHostIP(char* buf, uint8_t length) {
    auto handle = getRosNodeHandle();
    std::string address = handle->param("host_tcp_address", std::string("127.0.0.1"));

    return snprintf(buf, length, "%s", address.c_str());
}

LogLevel SettingsContainer::getLogLevel() {
    auto handle = getRosNodeHandle();
    std::string logLevel = handle->param("log_level", std::string("Info"));

    if (logLevel == "Debug") {
        return LogLevel::Debug;
    }
    if (logLevel == "Info") {
        return LogLevel::Info;
    }
    if (logLevel == "Warn") {
        return LogLevel::Warn;
    }
    if (logLevel == "Error") {
        return LogLevel::Error;
    }

    return LogLevel::Info;
}

uint16_t SettingsContainer::getBBZVMStepDelayMs() {
    auto handle = getRosNodeHandle();
    return handle->param("bbzvm_step_delay_ms", 10);
}
