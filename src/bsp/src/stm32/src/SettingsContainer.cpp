#include "bsp/SettingsContainer.h"
#include "DefaultSettings.h"
#include <cstring>

uint16_t SettingsContainer::getUUID() { return UUID; }

uint32_t SettingsContainer::getHostPort() { return HOST_PORT; }

uint8_t SettingsContainer::getHostIP(char* buf, uint8_t length) {
    return snprintf(buf, length, HOST_IP);
}

LogLevel SettingsContainer::getLogLevel() { return LOG_LEVEL; }
