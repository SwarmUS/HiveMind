#include "bittybuzz/BittyBuzzBytecode.h"
#include "mocks/LoggerInterfaceMock.h"
#include <array>
#include <cstring>
#include <gtest/gtest-death-test.h>
#include <gtest/gtest.h>

class BittyBuzzBytecodeTestFixture : public testing::Test {
  protected:
    std::array<uint8_t, 8> m_bytecodeArray;
    BittyBuzzBytecode* m_bytecode;
    LoggerInterfaceMock* m_loggerMock;
    int logCounter = 0;

    void SetUp() override {
        for (uint8_t i = 0; i < m_bytecodeArray.size(); i++) {
            m_bytecodeArray[i] = i;
        }
        m_loggerMock = new LoggerInterfaceMock(logCounter);
        m_bytecode =
            new BittyBuzzBytecode(*m_loggerMock, m_bytecodeArray.data(), m_bytecodeArray.size());
    }

    void TearDown() override { delete m_bytecode; }
};

TEST_F(BittyBuzzBytecodeTestFixture, BittyBuzzBytecode_getBytecodeFetchFunction_getFirstBytes) {
    // Given
    // Then
    bbzvm_bcode_fetch_fun fun = m_bytecode->getBytecodeFetchFunction();
    const uint8_t* buff = fun(0, 3);

    // Expect
    for (uint8_t i = 0; i < 3; i++) {
        EXPECT_EQ(buff[i], i);
    }
    EXPECT_EQ(logCounter, 0);
}

TEST_F(BittyBuzzBytecodeTestFixture, BittyBuzzBytecode_getBytecodeFetchFunction_getLastBytes) {
    // Given
    // Then
    bbzvm_bcode_fetch_fun fun = m_bytecode->getBytecodeFetchFunction();
    const uint8_t* buff = fun(m_bytecodeArray.size() - 3, 3);

    // Expect
    for (uint8_t i = 0; i < 3; i++) {
        EXPECT_EQ(buff[i], i + m_bytecodeArray.size() - 3);
    }

    EXPECT_EQ(logCounter, 0);
}

TEST_F(BittyBuzzBytecodeTestFixture,
       BittyBuzzBytecode_getBytecodeFetchFunction_getMoreThanFour_shouldLog) {
    // Given
    // Then
    bbzvm_bcode_fetch_fun fun = m_bytecode->getBytecodeFetchFunction();
    const uint8_t* buff = fun(0, 5);
    // Expect
    EXPECT_EQ(buff, m_bytecodeArray.data());
    EXPECT_EQ(logCounter, 1);
}

TEST_F(BittyBuzzBytecodeTestFixture,
       BittyBuzzBytecode_getBytecodeFetchFunction_getOutOfBound_shouldLog) {
    // Given
    // Then
    bbzvm_bcode_fetch_fun fun = m_bytecode->getBytecodeFetchFunction();
    const uint8_t* buff = fun(9999, 2);
    // Expect
    EXPECT_EQ(buff, m_bytecodeArray.data() + 9999);
    EXPECT_EQ(logCounter, 1);
}

TEST_F(BittyBuzzBytecodeTestFixture, BittyBuzzBytecode_getBytecodeLength_getLength) {
    // Given
    // Then
    uint8_t length = m_bytecode->getBytecodeLength();

    // Expect
    EXPECT_EQ(length, m_bytecodeArray.size());
}
