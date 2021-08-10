#ifndef __LOGGERINTERFACEMOCK_H_
#define __LOGGERINTERFACEMOCK_H_

#include <cstdarg>
#include <gmock/gmock.h>
#include <logger/ILogger.h>
#include <string>

class LoggerInterfaceMock : public ILogger {
  public:
    LoggerInterfaceMock() = default;

    ~LoggerInterfaceMock() override = default;

    LogRet log(LogLevel level, const char* format, ...) override {
        (void)level;
        (void)format;

        return LogRet::Ok;
    }

    LogRet log(LogLevel level, const char* format, va_list args) override {
        (void)level;
        (void)format;
        (void)args;
        return LogRet::Ok;
    }
};

#endif // __LOGGERINTERFACEMOCK_H_
