
#ifndef __LOGGERINTERFACEMOCK_H_
#define __LOGGERINTERFACEMOCK_H_

#include <gmock/gmock.h>
#include <logger/ILogger.h>
#include <string>

class LoggerInterfaceMock final : public ILogger {
  public:
    int& m_logCallCounter;
    std::string& m_logLastFormat;

    LoggerInterfaceMock(int& logCounter, std::string& logLastFormat) :
        m_logCallCounter(logCounter), {}
    ~LoggerInterfaceMock() override = default;

    LogRet log(LogLevel level, const char* format, ...) const override {
        (void)level;
        (void)format;

        m_logLastFormat = format;

        m_logCallCounter++;
        return LogRet::Ok;
    }
};

#endif // __LOGGERINTERFACEMOCK_H_
