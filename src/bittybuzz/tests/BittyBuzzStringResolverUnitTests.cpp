#include "bittybuzz/BittyBuzzStringResolver.h"
#include "mocks/LoggerInterfaceMock.h"

#include <array>
#include <gtest/gtest.h>

class BittyBuzzStringResolverTestFixture : public testing::Test {
  protected:
    BittyBuzzStringResolver* m_stringResolver;
    LoggerInterfaceMock* m_loggerMock;
    int logCounter = 0;
    std::string logLastFormat;
    std::array<std::pair<const uint16_t, const char*>, 3> m_array = {
        {{99, "hello"}, {100, "world"}, {101, "hi"}}};

    void SetUp() override {
        m_loggerMock = new LoggerInterfaceMock(logCounter, logLastFormat);
        m_stringResolver =
            new BittyBuzzStringResolver(m_array.data(), m_array.size(), 99, *m_loggerMock);
    }

    void TearDown() override {
        delete m_stringResolver;
        delete m_loggerMock;
    }
};

TEST_F(BittyBuzzStringResolverTestFixture, BittyBuzzStringResolver_getString_getValidId) {
    // Given
    uint16_t stringId = 100;
    // Then
    std::optional<const char*> ret = m_stringResolver->getString(stringId);

    // Expect
    EXPECT_EQ(logCounter, 0);
    EXPECT_STREQ(ret.value(), m_array[1].second);
    EXPECT_EQ(ret.operator bool(), true);
}

TEST_F(BittyBuzzStringResolverTestFixture, BittyBuzzStringResolver_getString_getFirstId) {
    // Given
    uint16_t stringId = 99;
    // Then
    std::optional<const char*> ret = m_stringResolver->getString(stringId);

    // Expect
    EXPECT_EQ(logCounter, 0);
    EXPECT_STREQ(ret.value(), m_array[0].second);
    EXPECT_EQ(ret.operator bool(), true);
}

TEST_F(BittyBuzzStringResolverTestFixture, BittyBuzzStringResolver_getString_getLastId) {
    // Given
    uint16_t stringId = 101;
    // Then
    std::optional<const char*> ret = m_stringResolver->getString(stringId);

    // Expect
    EXPECT_EQ(logCounter, 0);
    EXPECT_STREQ(ret.value(), m_array[2].second);
    EXPECT_EQ(ret.operator bool(), true);
}

TEST_F(BittyBuzzStringResolverTestFixture,
       BittyBuzzStringResolver_getString_getReservedId_getIdTooLow) {
    // Given
    uint16_t stringId = 90;
    // Then
    std::optional<const char*> ret = m_stringResolver->getString(stringId);

    // Expect
    EXPECT_EQ(logCounter, 0);
    EXPECT_EQ(ret.operator bool(), false);
}

TEST_F(BittyBuzzStringResolverTestFixture, BittyBuzzStringResolver_getString_getOutOfBoundId) {
    // Given
    uint16_t stringId = 102;
    // Then
    std::optional<const char*> ret = m_stringResolver->getString(stringId);

    // Expect
    EXPECT_EQ(logCounter, 0);
    EXPECT_EQ(ret.operator bool(), false);
}

TEST_F(BittyBuzzStringResolverTestFixture, BittyBuzzStringResolver_getString_getCorruptedId) {
    // Given
    uint16_t stringId = 100;

    // Recreating object since we can't edit m_array
    std::array<std::pair<const uint16_t, const char*>, 3> array = {
        {{98, "hello"}, {99, "world"}, {100, "hi"}}};

    BittyBuzzStringResolver stringResolver =
        BittyBuzzStringResolver(array.data(), array.size(), 99, *m_loggerMock);

    // Then
    std::optional<const char*> ret = stringResolver.getString(stringId);

    // Expect
    EXPECT_EQ(logCounter, 1);
    EXPECT_EQ(ret.operator bool(), false);
}
