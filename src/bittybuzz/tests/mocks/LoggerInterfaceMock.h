
#ifndef __LOGGERINTERFACEMOCK_H_
#define __LOGGERINTERFACEMOCK_H_

#include <cstdarg>
#include <gmock/gmock.h>
#include <logger/ILogger.h>
#include <string>

class LoggerInterfaceMock : public ILogger {
  public:
    int& m_logCallCounter;
    std::string& m_logLastFormat;

    LoggerInterfaceMock(int& logCounter, std::string& logLastFormat) :
        m_logCallCounter(logCounter), m_logLastFormat(logLastFormat) {}
    ~LoggerInterfaceMock() override = default;

    LogRet log(LogLevel level, const char* format, ...) override {

        va_list args;
        va_start(args, format);
        LogRet retValue = log(level, format, args);
        va_end(args);
        return retValue;
    }

    LogRet log(LogLevel level, const char* format, va_list args) override {
        (void)level;

        const int bufferSize = 1024;
        char buffer[bufferSize];
        vsnprintf(buffer, bufferSize, format, args);
        m_logLastFormat = std::string(buffer);
        m_logCallCounter++;

        return LogRet::Ok;
    }
};

#endif // __LOGGERINTERFACEMOCK_H_
