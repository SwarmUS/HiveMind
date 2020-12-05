#ifndef __LOGGER_H_
#define __LOGGER_H_

/*
 * @brief The log level used for the logger;
 */
enum LogLevel { DEBUG = 0, INFO = 1, WARN = 2, ERROR = 3 };

/*
 * @brief TODO: change if namespace
 */
class Logger {
  public:
    virtual ~Logger(){};
    virtual int log(LogLevel level, const char* format, ...) = 0;
};

#endif // __LOGGER_H_
