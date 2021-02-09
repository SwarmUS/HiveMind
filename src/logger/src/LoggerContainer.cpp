#include "logger/LoggerContainer.h"
#include <bsp/BSPContainer.h>
#include <bsp/SettingsContainer.h>

Logger& LoggerContainer::getLogger() {
    static Logger s_logger =
        Logger(SettingsContainer::getLogLevel(), BSPContainer::getUserInterface());
    return s_logger;
}
