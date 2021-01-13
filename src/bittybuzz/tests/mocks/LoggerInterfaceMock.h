
#ifndef __LOGGERINTERFACEMOCK_H_
#define __LOGGERINTERFACEMOCK_H_

#include <cstdarg>
#include <gmock/gmock.h>
#include <logger/ILogger.h>
#include <string>

class LoggerInterfaceMock final : public ILogger {
  public:
    int& m_logCallCounter;
    std::string& m_logLastFormat;

    LoggerInterfaceMock(int& logCounter, std::string& logLastFormat) :
        m_logCallCounter(logCounter), m_logLastFormat(logLastFormat) {}
    ~LoggerInterfaceMock() override = default;

    LogRet log(LogLevel level, const char* format, ...) const override {
        (void)level;
        const int bufferSize = 1024;
        char buffer[bufferSize];

        va_list args;
        va_start(args, format);

        vsnprintf(buffer, bufferSize, format, args);
        m_logLastFormat = std::string(buffer);

        va_end(args);

        m_logCallCounter++;
        return LogRet::Ok;
    }
};

#endif // __LOGGERINTERFACEMOCK_H_
