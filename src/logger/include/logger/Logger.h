#ifndef __LOGGER_H_
#define __LOGGER_H_

#include "logger/ILogger.h"
#include <bsp/IUserInterface.h>

#include <Mutex.h>

class Logger : public ILogger {
  public:
    Logger(LogLevel level, IUserInterface& ui);
    ~Logger() override = default;

    LogRet log(LogLevel level, const char* format, ...) override;

  private:
    static char logLevelToString(LogLevel logLevel);

    IUserInterface& m_ui;
    LogLevel m_logLevel;
};

#endif // __LOGGER_H_
