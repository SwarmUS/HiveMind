#ifndef __LOGGER_IMPL_H_
#define __LOGGER_IMPL_H_

#include "logger/logger.h"
#include <bsp/ui.h>

#include <FreeRTOS.h>
#include <semphr.h>

/*
 * @brief TODO: change if namespace
 */
class LoggerImpl : public Logger {
  public:
    LoggerImpl(LogLevel level, UI* ui);
    ~LoggerImpl() override {}

    LogRet log(LogLevel level, const char* format, ...) override;

  private:
    SemaphoreHandle_t m_semaphore;
    LogLevel m_log_level;
    UI* m_ui;
};

#endif // __LOGGER_IMPL_H_
