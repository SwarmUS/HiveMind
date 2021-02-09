#ifndef __DEFAULTSETTINGS_H__
#define __DEFAULTSETTINGS_H__
// clang-format off

#include <logger/ILogger.h>

/**
 * @brief Template file used by CMake to provide default or user-provided values for the settings
 */

#define UUID @UUID@
#define HOST_PORT @HOST_PORT@
#define HOST_IP "@HOST_IP@"
#define LOG_LEVEL (LogLevel::@LOG_LEVEL@)

// clang-format on
#endif //__DEFAULTSETTINGS_H__
