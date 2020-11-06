#include <gtest/gtest.h>


class HalTestFixture : public testing::Test {

};

TEST_F(HalTestFixture, Register_Read_Address1_Returns1) {
    ASSERT_EQ(1, 2);
}

