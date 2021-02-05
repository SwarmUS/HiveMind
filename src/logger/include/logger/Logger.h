#ifndef __LOGGER_H_
#define __LOGGER_H_

#include "logger/ILogger.h"
#include <bsp/IUserInterface.h>

#include <FreeRTOS.h>
#include <freertos-utils/Mutex.h>
#include <semphr.h>

class Logger : public ILogger {
  public:
    Logger(LogLevel level, const IUserInterface& ui);
    ~Logger() override = default;

    LogRet log(LogLevel level, const char* format, ...) override;

  private:
    const IUserInterface& m_ui;
    Mutex m_mutex;
    LogLevel m_logLevel;
};

#endif // __LOGGER_H_
