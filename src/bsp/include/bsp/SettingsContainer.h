#ifndef __SETTINGSCONTAINER_H__
#define __SETTINGSCONTAINER_H__

#include <cstdint>
#include <string>

#define MAX_IP_LENGTH 16

namespace SettingsContainer {
    /**
     * @brief Returns the UUID of the board
     * @return
     */
    uint16_t GetUUID();

    /**
     * @brief Returns the TCP port on which the host has opened a port
     * @return
     */
    uint32_t GetHostPort();

    /**
     * @brief Copies the host IP address into the given buffer
     * @param buf Buffer in which to copy the IP
     * @param length Length of the buffer in bytes
     * @return number of bytes written or 0 if there was an error
     */
    uint8_t GetHostIP(char* buf, uint8_t length);
} // namespace SettingsContainer

#endif //__SETTINGSCONTAINER_H__
