#include "common/circular_buff.h"
#include <gtest/gtest.h>

static const uint8_t size = 8;

class CircularBuffFixture : public testing::Test {
  public:
    CircularBuff m_circularBuff;
    uint8_t m_data[size];
    void SetUp() override { CircularBuff_init(&m_circularBuff, m_data, size); }
    void TearDown() override {}
};

TEST_F(CircularBuffFixture, CircularBuff_Init_Success) {
    // Given
    // Then
    bool ret = CircularBuff_init(&m_circularBuff, m_data, size);

    // Expect
    EXPECT_EQ(ret, true);
}

TEST_F(CircularBuffFixture, CircularBuff_Init_NullCircularBuff) {
    // Given
    // Then
    bool ret = CircularBuff_init(NULL, m_data, size);

    // Expect
    EXPECT_EQ(ret, false);
}

TEST_F(CircularBuffFixture, CircularBuff_Init_NullData) {
    // Given
    // Then
    bool ret = CircularBuff_init(&m_circularBuff, NULL, size);

    // Expect
    EXPECT_EQ(ret, false);
}


TEST_F(CircularBuffFixture, CircularBuff_getFreeSize_Empty) {
    // Given
    // Then
    uint16_t ret = CircularBuff_getFreeSize(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, 8);
}

TEST_F(CircularBuffFixture, CircularBuff_getFreeSize_Full) {
    // Given
    m_circularBuff.writePos = size;

    // Then
    uint16_t ret = CircularBuff_getFreeSize(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, 0);
}


TEST_F(CircularBuffFixture, CircularBuff_getFreeSize_ReadHigherThanWrite) {
    // Given
    m_circularBuff.readPos = 5;
    m_circularBuff.writePos = 4;

    // Then
    uint16_t ret = CircularBuff_getFreeSize(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, 1);
}


TEST_F(CircularBuffFixture, CircularBuff_getFreeSize_WriteHigherThanRead) {
    // Given
    m_circularBuff.readPos = 4;
    m_circularBuff.writePos = 5;

    // Then
    uint16_t ret = CircularBuff_getFreeSize(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, size-1);
}

TEST_F(CircularBuffFixture, CircularBuff_getLength_Empty) {
    // Given
    // Then
    uint16_t ret = CircularBuff_getLength(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, 0);
}

TEST_F(CircularBuffFixture, CircularBuff_getLength_Full) {
    // Given
    m_circularBuff.writePos = size;

    // Then
    uint16_t ret = CircularBuff_getLength(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, size);
}

TEST_F(CircularBuffFixture, CircularBuff_getLength_WriteHigherThanRead) {
    // Given
    m_circularBuff.readPos = 4;
    m_circularBuff.writePos = 5;

    // Then
    uint16_t ret = CircularBuff_getLength(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, 1);
}

TEST_F(CircularBuffFixture, CircularBuff_getLength_ReadHigherThanWrite) {
    // Given
    m_circularBuff.readPos = 5;
    m_circularBuff.writePos = 4;

    // Then
    uint16_t ret = CircularBuff_getLength(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, size-1);
}

TEST_F(CircularBuffFixture, CircularBuff_isFull_Empty) {
    // Given
    // Then
    bool ret = CircularBuff_isFull(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, false);
}

TEST_F(CircularBuffFixture, CircularBuff_isFull_Full) {
    // Given
    m_circularBuff.writePos = size;

    // Then
    bool ret = CircularBuff_isFull(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, true);
}

TEST_F(CircularBuffFixture, CircularBuff_isFull_ReadHigherThanWrite) {
    // Given
    m_circularBuff.readPos = 5;
    m_circularBuff.writePos = 4;

    // Then
    bool ret = CircularBuff_isFull(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, false);
}

TEST_F(CircularBuffFixture, CircularBuff_isFull_WriteHigherThanRead) {
    // Given
    m_circularBuff.readPos = 4;
    m_circularBuff.writePos = 5;

    // Then
    bool ret = CircularBuff_isFull(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, false);
}

TEST_F(CircularBuffFixture, CircularBuff_putc_Empty) {
    // Given
    // Then
    CircularBuffRet ret = CircularBuff_putc(&m_circularBuff, 42);

    // Expect
    EXPECT_EQ(ret, CircularBuff_Ret_Ok);
    EXPECT_EQ(m_circularBuff.writePos, 1);
    EXPECT_EQ(m_circularBuff.data[0], 42);
}

TEST_F(CircularBuffFixture, CircularBuff_putc_Full) {
    // Given
    m_circularBuff.writePos = size;
    // Then6
    CircularBuffRet ret = CircularBuff_putc(&m_circularBuff, 42);

    // Expect
    EXPECT_EQ(m_circularBuff.writePos, 1);
    EXPECT_EQ(m_circularBuff.data[0], 42);
    EXPECT_EQ(ret, CircularBuff_Ret_SpaceErr);
}

TEST_F(CircularBuffFixture, CircularBuff_putc_Loop) {
    // Given
    uint8_t read;
    CircularBuff_put(&m_circularBuff, m_data, size);
    CircularBuff_readc(&m_circularBuff, &read);

    // Then
    CircularBuffRet ret = CircularBuff_putc(&m_circularBuff, 42);

    // Expect
    EXPECT_EQ(m_circularBuff.writePos, 1);
    EXPECT_EQ(m_circularBuff.data[size], 42);
    EXPECT_EQ(ret, CircularBuff_Ret_Ok);
}

TEST_F(CircularBuffFixture, CircularBuff_put_Empty) {
    // Given
    memset(m_data, 42, size);

    // Then
    CircularBuffRet ret = CircularBuff_put(&m_circularBuff, m_data, (int)size/2);

    // Expect
    EXPECT_EQ(m_circularBuff.writePos, (int)size/2);
    EXPECT_EQ(m_circularBuff.data[size/2 - 1], 42);
    EXPECT_EQ(m_circularBuff.data[size/2], 0);
    EXPECT_EQ(ret, CircularBuff_Ret_Ok);
}