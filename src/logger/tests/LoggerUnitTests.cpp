#include "logger/ILogger.h"
#include "logger/Logger.h"
#include "mocks/UserInterfaceMock.h"
#include <gtest/gtest.h>

class LoggerTestFixture : public testing::Test {
  protected:
    UserInterfaceMock* m_uiMock;
    Logger* m_logger;
    void SetUp() override {
        int printCounter = 0;
        m_uiMock = new UserInterfaceMock(printCounter);
        m_logger = new Logger(LogLevel::Info, *m_uiMock);
    }
    void TearDown() override {
        delete m_uiMock;
        delete m_logger;
    }
};

// Since gmock doesn't have the ability to mock variable argument, we use a counter instead of
// using EXPECT_CALL
TEST_F(LoggerTestFixture, Logger_Log_Same_Level_Log_Once) {
    // Given
    // Then
    LogRet ret = m_logger->log(LogLevel::Info, "Info log");

    // Expect
    EXPECT_EQ(m_uiMock->m_printCallCounter, 2); // 2 since we print the log level
    EXPECT_EQ(ret, LogRet::Ok);
}

TEST_F(LoggerTestFixture, Logger_Log_Higher_Level_Log_Once) {
    // Given
    // Then
    LogRet ret = m_logger->log(LogLevel::Error, "Error log");

    // Expect
    EXPECT_EQ(m_uiMock->m_printCallCounter, 2); // 2 since we print the log level
    EXPECT_EQ(ret, LogRet::Ok);
}

TEST_F(LoggerTestFixture, Logger_Log_Lower_Level_No_Log) {
    // Given
    // Then
    LogRet ret = m_logger->log(LogLevel::Debug, "Debug log");

    // Expect
    EXPECT_EQ(m_uiMock->m_printCallCounter, 0);
    EXPECT_EQ(ret, LogRet::LowLevel);
}

TEST_F(LoggerTestFixture, Logger_Log_ReleaseSemaphore) {
    // Given
    // Then
    LogRet ret = m_logger->log(LogLevel::Error, "Debug log");

    // Expect
    EXPECT_EQ(m_uiMock->m_printCallCounter, 2);
    EXPECT_EQ(ret, LogRet::Ok);

    // Call a second time and see if it's a success
    ret = m_logger->log(LogLevel::Error, "Error log");
    EXPECT_EQ(ret, LogRet::Ok);
    EXPECT_EQ(m_uiMock->m_printCallCounter, 4);
}
