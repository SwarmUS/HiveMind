#ifndef __SETTINGSCONTAINER_H__
#define __SETTINGSCONTAINER_H__

#include <stdint.h>
#include <string>

namespace SettingsContainer {
    uint16_t GetUUID();

    uint32_t GetHostPort();

    std::string GetHostIP();
} // namespace SettingsContainer

#endif //__SETTINGSCONTAINER_H__
