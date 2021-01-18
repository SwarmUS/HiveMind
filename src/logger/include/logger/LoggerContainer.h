#ifndef __LOGGERCONTAINER_H_
#define __LOGGERCONTAINER_H_

#include "Logger.h"

namespace LoggerContainer {

    /**
     *@brief Return a reference to the global logger
     **/
    Logger& getLogger();

} // namespace LoggerContainer

#endif // __LOGGERCONTAINER_H_
