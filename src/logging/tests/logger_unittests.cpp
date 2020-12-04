#include <gtest/gtest.h>

class LoggerTestFixture : public testing::Test {
  protected:
    virtual void SetUp() {}
    virtual void TearDown() {}
};

TEST_F(LoggerTestFixture, Logger_Log_Same_Level) {}
