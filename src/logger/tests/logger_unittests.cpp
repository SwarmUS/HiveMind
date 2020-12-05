#include "logger/logger.h"
#include "logger/logger_impl.h"
#include "mocks/ui_mock.h"
#include <gtest/gtest.h>

class LoggerTestFixture : public testing::Test {
  protected:
    UIMock ui_mock;
    Logger* logger;
    virtual void SetUp() { logger = new LoggerImpl(INFO, &ui_mock); }
    virtual void TearDown() { delete logger; }
};

TEST_F(LoggerTestFixture, Logger_Log_Same_Level_Log_Once) {

    // Given

    // Then
    logger->log(INFO, "Info log");

    // Expect
    EXPECT_EQ(ui_mock.print_called, 1);
}

TEST_F(LoggerTestFixture, Logger_Log_Higher_Level_Log_Once) {

    // Given

    // Then
    logger->log(ERROR, "Error log");

    // Expect
    EXPECT_EQ(ui_mock.print_called, 1);
}

TEST_F(LoggerTestFixture, Logger_Log_Lower_Level_No_Log) {

    // Given

    // Then
    logger->log(DEBUG, "Debug log");

    // Expect
    EXPECT_EQ(ui_mock.print_called, 0);
}
