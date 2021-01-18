#include "logger/LoggerContainer.h"
#include <bsp/BSPContainer.h>

Logger& LoggerContainer::getLogger() {
    // TODO: Inject log level, from bsp maybe? TBD
    static Logger s_logger = Logger(LogLevel::Info, BSPContainer::getUserInterface());
    return s_logger;
}
