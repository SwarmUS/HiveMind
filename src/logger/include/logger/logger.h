#ifndef __LOGGER_H_
#define __LOGGER_H_

/**
 * @brief The log level used for the logger;
 */
enum class LogLevel { Debug = 0, Info = 1, Warn = 2, Error = 3 };

/**
 * @brief The return value of the logger
 */
enum class LogRet { Ok = 0, LowLevel = 1, Error = 2 };

/**
 * @brief A logger class with basic logging cappibilities
 */
class Logger {
  public:
    virtual ~Logger(){};

    /**
     * @brief Logs if the provided level is higher than the current log level
     *
     * @param [in] level the log level of the current call
     *
     * @param [in] format Text to be written, can contain format specifiers that will be replace by
     *values specified in the additionnal arguments, matches the standard printf function
     *
     * @param [in] ... Additionnal arguments for the format parameter
     *
     * @return Returns Ok on success, LowLevel if the level provided is lower than the current one
     *and Error if an error occured
     */
    virtual int log(LogLevel level, const char* format, ...) = 0;
};

#endif // __LOGGER_H_
