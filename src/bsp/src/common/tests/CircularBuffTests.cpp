#include "common/circular_buff.h"
#include <bits/stdint-uintn.h>
#include <gtest/gtest.h>

static const uint8_t size = 8;

class CircularBuffFixture : public testing::Test {
  public:
    CircularBuff m_circularBuff;
    uint8_t m_data[size];
    void SetUp() override {
        memset(m_data, 0, size);
        CircularBuff_init(&m_circularBuff, m_data, size);
    }
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
    m_circularBuff.m_isFull = true;

    // Then
    uint16_t ret = CircularBuff_getFreeSize(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, 0);
}

TEST_F(CircularBuffFixture, CircularBuff_getFreeSize_ReadHigherThanWrite) {
    // Given
    m_circularBuff.m_readPos = 5;
    m_circularBuff.m_writePos = 4;

    // Then
    uint16_t ret = CircularBuff_getFreeSize(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, 1);
}

TEST_F(CircularBuffFixture, CircularBuff_getFreeSize_WriteHigherThanRead) {
    // Given
    m_circularBuff.m_readPos = 4;
    m_circularBuff.m_writePos = 5;

    // Then
    uint16_t ret = CircularBuff_getFreeSize(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, size - 1);
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
    m_circularBuff.m_isFull = true;

    // Then
    uint16_t ret = CircularBuff_getLength(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, size);
}

TEST_F(CircularBuffFixture, CircularBuff_getLength_WriteHigherThanRead) {
    // Given
    m_circularBuff.m_readPos = 4;
    m_circularBuff.m_writePos = 5;

    // Then
    uint16_t ret = CircularBuff_getLength(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, 1);
}

TEST_F(CircularBuffFixture, CircularBuff_getLength_ReadHigherThanWrite) {
    // Given
    m_circularBuff.m_readPos = 5;
    m_circularBuff.m_writePos = 4;

    // Then
    uint16_t ret = CircularBuff_getLength(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, size - 1);
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
    m_circularBuff.m_isFull = true;

    // Then
    bool ret = CircularBuff_isFull(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, true);
}

TEST_F(CircularBuffFixture, CircularBuff_isFull_ReadHigherThanWrite) {
    // Given
    m_circularBuff.m_readPos = 5;
    m_circularBuff.m_writePos = 4;

    // Then
    bool ret = CircularBuff_isFull(&m_circularBuff);

    // Expect
    EXPECT_EQ(ret, false);
}

TEST_F(CircularBuffFixture, CircularBuff_isFull_WriteHigherThanRead) {
    // Given
    m_circularBuff.m_readPos = 4;
    m_circularBuff.m_writePos = 5;

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
    EXPECT_EQ(m_circularBuff.m_writePos, 1);
    EXPECT_EQ(m_circularBuff.m_data[0], 42);
}

TEST_F(CircularBuffFixture, CircularBuff_putc_Full) {
    // Given
    m_circularBuff.m_isFull = true;
    // Then6
    CircularBuffRet ret = CircularBuff_putc(&m_circularBuff, 42);

    // Expect
    EXPECT_EQ(ret, CircularBuff_Ret_SpaceErr);
}

TEST_F(CircularBuffFixture, CircularBuff_putc_LoopBuff) {
    // Given
    m_circularBuff.m_readPos = 5;
    m_circularBuff.m_writePos = 8;

    // Then
    CircularBuffRet ret = CircularBuff_putc(&m_circularBuff, 42);

    // Expect
    EXPECT_EQ(m_circularBuff.m_writePos, 1);
    EXPECT_EQ(m_circularBuff.m_data[0], 42);
    EXPECT_EQ(ret, CircularBuff_Ret_Ok);
}

TEST_F(CircularBuffFixture, CircularBuff_put_Empty) {
    // Given
    uint8_t data[size];
    memset(data, 42, size);

    // Then
    CircularBuffRet ret = CircularBuff_put(&m_circularBuff, data, (int)(size / 2));

    // Expect
    EXPECT_EQ(m_circularBuff.m_writePos, (int)(size / 2));
    EXPECT_EQ(m_circularBuff.m_data[size / 2 - 1], 42);
    EXPECT_EQ(m_circularBuff.m_data[size / 2], 0);
    EXPECT_EQ(ret, CircularBuff_Ret_Ok);
}

TEST_F(CircularBuffFixture, CircularBuff_put_Full) {
    // Given
    m_circularBuff.m_isFull = true;
    uint8_t data[size];
    memset(data, 42, size);

    // Then
    CircularBuffRet ret = CircularBuff_put(&m_circularBuff, data, (int)(size / 2));

    // Expect
    EXPECT_EQ(m_circularBuff.m_data[size / 2], 0);
    EXPECT_EQ(ret, CircularBuff_Ret_SpaceErr);
}

TEST_F(CircularBuffFixture, CircularBuff_put_NotEnoughSpace) {
    // Given
    m_circularBuff.m_writePos = 1;
    uint8_t data[size];
    memset(data, 42, size);

    // Then
    CircularBuffRet ret = CircularBuff_put(&m_circularBuff, data, (int)size);

    // Expect
    EXPECT_EQ(m_circularBuff.m_data[size / 2], 0);
    EXPECT_EQ(ret, CircularBuff_Ret_SpaceErr);
}

TEST_F(CircularBuffFixture, CircularBuff_put_Loops) {
    // Given
    m_circularBuff.m_writePos = 5;
    m_circularBuff.m_readPos = 4;
    uint8_t data[size];
    memset(data, 42, size);

    // Then
    CircularBuffRet ret = CircularBuff_put(&m_circularBuff, data, (int)(size / 2));

    // Expect
    EXPECT_EQ(m_circularBuff.m_data[5], 42);
    EXPECT_EQ(m_circularBuff.m_writePos, (5 + size / 2) % size);
    EXPECT_EQ(ret, CircularBuff_Ret_Ok);
}

TEST_F(CircularBuffFixture, CircularBuff_getc_Empty) {
    // Given
    uint8_t data = 0;
    m_circularBuff.m_data[0] = 42;
    // Then
    CircularBuffRet ret = CircularBuff_getc(&m_circularBuff, &data);

    // Expect
    EXPECT_EQ(ret, CircularBuff_Ret_EmptyErr);
    EXPECT_EQ(data, 0);
}

TEST_F(CircularBuffFixture, CircularBuff_getc_Full) {
    // Given
    uint8_t data = 0;
    m_circularBuff.m_data[0] = 42;
    m_circularBuff.m_isFull = true;
    // Then
    CircularBuffRet ret = CircularBuff_getc(&m_circularBuff, &data);

    // Expect
    EXPECT_EQ(ret, CircularBuff_Ret_Ok);
    EXPECT_EQ(data, 42);
    EXPECT_EQ(m_circularBuff.m_isFull, false);
}

TEST_F(CircularBuffFixture, CircularBuff_getc_LoopBuff) {
    // Given
    uint8_t data = 0;
    m_circularBuff.m_data[size - 1] = 42;
    m_circularBuff.m_readPos = 8;
    m_circularBuff.m_writePos = 5;

    // Then
    CircularBuffRet ret = CircularBuff_getc(&m_circularBuff, &data);

    // Expect
    EXPECT_EQ(m_circularBuff.m_readPos, 1);
    EXPECT_EQ(m_circularBuff.m_data[size - 1], 42);
    EXPECT_EQ(ret, CircularBuff_Ret_Ok);
}

TEST_F(CircularBuffFixture, CircularBuff_get_Empty) {
    // Given
    uint8_t data[size];

    // Then
    uint16_t ret = CircularBuff_get(&m_circularBuff, data, (int)(size / 2));

    // Expect
    EXPECT_EQ(ret, 0);
    EXPECT_EQ(m_circularBuff.m_readPos, 0);
}

TEST_F(CircularBuffFixture, CircularBuff_get_Full) {
    // Given
    m_circularBuff.m_isFull = true;
    uint8_t data[size];
    memset(m_circularBuff.m_data, 42, size);

    // Then
    uint16_t ret = CircularBuff_get(&m_circularBuff, data, (int)(size / 2));

    // Expect
    EXPECT_EQ(m_circularBuff.m_data[0], 42);
    EXPECT_EQ(ret, (int)(size / 2));
    EXPECT_EQ(m_circularBuff.m_isFull, false);
    EXPECT_EQ(m_circularBuff.m_readPos, (int)(size / 2));
}

TEST_F(CircularBuffFixture, CircularBuff_get_Loops) {
    // Given
    m_circularBuff.m_writePos = 4;
    m_circularBuff.m_readPos = 5;
    uint8_t data[size];
    memset(m_circularBuff.m_data, 42, size);

    // Then
    uint16_t ret = CircularBuff_get(&m_circularBuff, data, (int)(size / 2));

    // Expect
    EXPECT_EQ(m_circularBuff.m_data[5], 42);
    EXPECT_EQ(m_circularBuff.m_readPos, (5 + size / 2) % size);
    EXPECT_EQ(ret, (int)(size / 2));
}

// SUTFF HERE
//

TEST_F(CircularBuffFixture, CircularBuff_getZeroCopy_Empty) {
    // Given

    // Then
    ZeroCopyBuff ret = CircularBuff_getZeroCopy(&m_circularBuff, (int)(size / 2));

    // Expect
    EXPECT_EQ(ret.length, 0);
    EXPECT_EQ(ret.status, CircularBuff_Ret_EmptyErr);
    EXPECT_EQ(ret.data, m_circularBuff.m_data);
    EXPECT_EQ(m_circularBuff.m_readPos, 0);
}

TEST_F(CircularBuffFixture, CircularBuff_getZeroCopy_Full) {
    // Given
    m_circularBuff.m_isFull = true;
    m_circularBuff.m_readPos = 1;
    m_circularBuff.m_writePos = 1;
    uint16_t readSize = (int)(size / 2);

    // Then
    ZeroCopyBuff ret = CircularBuff_getZeroCopy(&m_circularBuff, readSize);

    // Expect
    EXPECT_EQ(ret.length, readSize);
    EXPECT_EQ(ret.status, CircularBuff_Ret_Ok);
    EXPECT_EQ(ret.data, m_circularBuff.m_data + 1);
    EXPECT_EQ(m_circularBuff.m_readPos, readSize + 1);
    EXPECT_EQ(m_circularBuff.m_isFull, false);
}

TEST_F(CircularBuffFixture, CircularBuff_getZeroCopy_FullReadZero) {
    // Given
    m_circularBuff.m_isFull = true;
    m_circularBuff.m_readPos = 1;
    m_circularBuff.m_writePos = 1;
    uint16_t readSize = 0;

    // Then
    ZeroCopyBuff ret = CircularBuff_getZeroCopy(&m_circularBuff, readSize);

    // Expect
    EXPECT_EQ(ret.length, readSize);
    EXPECT_EQ(ret.status, CircularBuff_Ret_Ok);
    EXPECT_EQ(ret.data, m_circularBuff.m_data + 1);
    EXPECT_EQ(m_circularBuff.m_readPos, 1);
    EXPECT_EQ(m_circularBuff.m_isFull, true);
}

TEST_F(CircularBuffFixture, CircularBuff_getZeroCopy_Loops) {
    // Given
    m_circularBuff.m_writePos = 5;
    m_circularBuff.m_readPos = 6;
    m_circularBuff.m_isFull = true;
    uint16_t readSize = (int)(size / 2);

    // Then
    ZeroCopyBuff ret = CircularBuff_getZeroCopy(&m_circularBuff, readSize);

    // Expect
    EXPECT_EQ(ret.length, size - 6);
    EXPECT_EQ(ret.status, CircularBuff_Ret_Ok);
    EXPECT_EQ(ret.data, m_circularBuff.m_data + 6);
    EXPECT_EQ(m_circularBuff.m_readPos, 8);
    EXPECT_EQ(m_circularBuff.m_isFull, false);
}

// END STUFF
TEST_F(CircularBuffFixture, CircularBuff_clear_ClearBuff) {
    // Given
    m_circularBuff.m_writePos = 4;
    m_circularBuff.m_readPos = 5;

    // Then
    bool ret = CircularBuff_clear(&m_circularBuff);

    // Expect
    EXPECT_EQ(m_circularBuff.m_readPos, 0);
    EXPECT_EQ(m_circularBuff.m_writePos, 0);
    EXPECT_EQ(ret, true);
}
