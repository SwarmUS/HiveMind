#include <gmock/gmock.h>
#include <logger/ILogger.h>

class LoggerInterfaceMock final : public ILogger {
  public:
    int& m_logCallCounter;

    LoggerInterfaceMock(int& logCounter) : m_logCallCounter(logCounter) {}
    ~LoggerInterfaceMock() override = default;

    LogRet log(LogLevel level, const char* format, ...) const override {
        (void) level;
        (void) format;
        
        m_logCallCounter++;
        return LogRet::Ok;
    }
};
