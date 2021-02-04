#include "bsp/SettingsContainer.h"
#include "DefaultSettings.h"
#include <cstring>

uint16_t SettingsContainer::GetUUID() { return UUID; }

uint32_t SettingsContainer::GetHostPort() { return HOST_PORT; }

uint8_t SettingsContainer::GetHostIP(char* buf, uint8_t length) {
    return snprintf(buf, length, HOST_IP);
}