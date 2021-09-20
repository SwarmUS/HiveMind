#include "bsp/SettingsContainer.h"
#include "DefaultSettings.h"

uint32_t SettingsContainer::getHostPort() { return HOST_PORT; }

uint8_t SettingsContainer::getHostIP(char* buf, uint8_t length) {
    return snprintf(buf, length, HOST_IP);
}

LogLevel SettingsContainer::getLogLevel() { return LOG_LEVEL; }

uint16_t SettingsContainer::getBBZVMStepDelay() { return BBZVM_STEP_DELAY; }
