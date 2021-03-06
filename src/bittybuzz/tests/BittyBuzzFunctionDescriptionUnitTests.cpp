#include <bittybuzz/BittyBuzzFunctionDescription.h>
#include <gtest/gtest.h>

class BittyBuzzFunctionDescriptionTestFixture : public testing::Test {
  protected:
    BittyBuzzFunctionDescription m_functionDescription;

    void SetUp() override {}

    void TearDown() override {}
};

TEST_F(BittyBuzzFunctionDescriptionTestFixture,
       BittyBuzzFunctionDescription_addArgument_stillSpace) {
    // Given

    // Then
    bool ret = m_functionDescription.addArgument("arg", FunctionDescriptionArgumentTypeDTO::Int);

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(BittyBuzzFunctionDescriptionTestFixture, BittyBuzzFunctionDescription_addArgument_noSpace) {
    // Given
    // Fulling it
    for (uint i = 0; i < m_functionDescription.getArguments().size(); i++) {
        m_functionDescription.addArgument("arg", FunctionDescriptionArgumentTypeDTO::Int);
    }

    // Then
    bool ret = m_functionDescription.addArgument("arg", FunctionDescriptionArgumentTypeDTO::Int);

    // Expect
    EXPECT_FALSE(ret);
}
