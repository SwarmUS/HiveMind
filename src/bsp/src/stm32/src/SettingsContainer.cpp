#include "bsp/SettingsContainer.h"
#include "DefaultSettings.h"

uint16_t SettingsContainer::GetUUID() { return UUID; }

uint32_t SettingsContainer::GetHostPort() { return HOST_PORT; }

std::string SettingsContainer::GetHostIP() { return HOST_IP; }