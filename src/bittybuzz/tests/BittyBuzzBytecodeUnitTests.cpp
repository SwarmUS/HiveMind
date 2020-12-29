#include "bittybuzz/BittyBuzzBytecode.h"
#include <gtest/gtest.h>

class BittyBuzzBytecodeTestFixture : public testing::Test {
  protected:
    friend class Logger;
    BittyBuzzBytecode* m_bytecode;
    void SetUp() override {}
    void TearDown() override {}
};
