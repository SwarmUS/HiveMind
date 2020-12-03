#ifndef __LOGGING_H_
#define __LOGGING_H_

#include <FreeRTOS.h>
#include <semphr.h>

/*
 * @brief The log level used for the logger;
 */
enum log_level_t { DEBUG, INFO, WARN, ERROR };

/*
 * @brief TODO: change if namespace
 */
class Logger {
  public:
    Logger(log_level_t level);
    static inline int log(log_level_t level, const char* format, ...);

  private:
    static SemaphoreHandle_t semaphore;
    static log_level_t log_level;
};

#endif // __LOGGING_H_
