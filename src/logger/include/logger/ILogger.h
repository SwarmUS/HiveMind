#ifndef __ILOGGER_H_
#define __ILOGGER_H_

/**
 * @brief The log level used for the logger
 */
enum class LogLevel {
    /** Log level to debug, prints everything*/
    Debug = 0,
    /** Log level to show basic info on state, etc*/
    Info = 1,
    /** Log level for warnings that don't cause application crash*/
    Warn = 2,
    /** Log level for unrecoverable error*/
    Error = 3
};

/**
 * @brief The return value of the logger
 */
enum class LogRet {
    /** Success return value */
    Ok = 0,
    /** Didn't log since the level of the logger is higher than the one provided in the function*/
    LowLevel = 1,
    /** An external error ocurred and the log failed */
    Error = 2
};

/**
 * @brief A logger class with basic logging capabilities
 */
class ILogger {
  public:
    virtual ~ILogger() = default;

    /**
     * @brief Logs if the provided level is higher than the current log level (Thread-safe)
     *
     * @param [in] level the log level of the current call
     *
     * @param [in] format Text to be written, can contain format specifiers that will be replaced by
     *values specified in the additionnal arguments, matches the standard printf function
     *
     * @param [in] ... Additionnal arguments for the format parameter
     *
     */
    virtual LogRet log(LogLevel level, const char* format, ...) = 0;
};

#endif // __ILOGGER_H_
