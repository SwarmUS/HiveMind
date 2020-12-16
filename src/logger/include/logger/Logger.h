#ifndef __LOGGER_H_
#define __LOGGER_H_

#include "logger/ILogger.h"
#include <bsp/IUserInterface.h>

#include <FreeRTOS.h>
#include <semphr.h>

class Logger : public ILogger {
  public:
    Logger(LogLevel level, const IUserInterface& ui);
    ~Logger() override;

    LogRet log(LogLevel level, const char* format, ...) const override;

  private:
    SemaphoreHandle_t m_semaphore;
    LogLevel m_logLevel;
    const IUserInterface& m_ui;
};

#endif // __LOGGER_H_
