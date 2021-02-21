#include "bittybuzz/BittyBuzzFunctionRegister.h"
#include <gtest/gtest.h>

class BittyBuzzFunctionRegisterTestFixture : public testing::Test {
  protected:
    BittyBuzzFunctionRegister* m_functionRegister;
    void SetUp() override {
        m_functionRegister = new BittyBuzzFunctionRegister();
    }

    void TearDown() override {
        delete m_functionRegister;
    }
};

TEST_F(BittyBuzzFunctionRegisterTestFixture, BittyBuzzFunctionRegister_registerFunction_empty) {
    // Given

    // Then
    bool ret = m_functionRegister->registerFunction("Hello World", 42);

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(BittyBuzzFunctionRegisterTestFixture, BittyBuzzFunctionRegister_registerFunction_full) {
    // Given

    // Then

    for (uint16_t i = 0; i < BittyBuzzFunctionRegister::m_maxSize; i++) {
        bool ret = m_functionRegister->registerFunction("Hello World", i);
        EXPECT_TRUE(ret);
    }

    bool ret = m_functionRegister->registerFunction("Hello World", 42);

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(BittyBuzzFunctionRegisterTestFixture, BittyBuzzFunctionRegister_getFunctionId_empty) {
    // Given

    // Then

    std::optional<uint16_t> ret = m_functionRegister->getFunctionId("Hello World");

    // Expect
    EXPECT_FALSE(ret.operator bool());
}

TEST_F(BittyBuzzFunctionRegisterTestFixture, BittyBuzzFunctionRegister_getFunctionId_exist) {
    // Given
    m_functionRegister->registerFunction("Hello World", 42);

    // Then

    std::optional<uint16_t> ret = m_functionRegister->getFunctionId("Hello World");

    // Expect
    EXPECT_TRUE(ret.operator bool());
    EXPECT_EQ(42, ret.value());
}

TEST_F(BittyBuzzFunctionRegisterTestFixture, BittyBuzzFunctionRegister_getFunctionId_doesNotExist) {
    // Given
    m_functionRegister->registerFunction("Hello World", 42);

    // Then

    std::optional<uint16_t> ret = m_functionRegister->getFunctionId("Good bye");

    // Expect
    EXPECT_FALSE(ret.operator bool());
}
