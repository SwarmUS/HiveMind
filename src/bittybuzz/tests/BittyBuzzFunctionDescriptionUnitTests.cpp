#include <bittybuzz/BittyBuzzFunctionDescription.h>
#include <gtest/gtest.h>

class BittyBuzzFunctionDescriptionTestFixture : public testing::Test {
  protected:
    const char* m_functionName = "functionName";
    BittyBuzzFunctionDescription* m_functionDescription;

    void SetUp() override {
        m_functionDescription = new BittyBuzzFunctionDescription(m_functionName);
    }

    void TearDown() override { delete m_functionDescription; }
};

TEST_F(BittyBuzzFunctionDescriptionTestFixture,
       BittyBuzzFunctionDescription_addArgument_stillSpace) {
    // Given

    // Then
    bool ret = m_functionDescription->addArgument("arg", FunctionDescriptionArgumentTypeDTO::Int);

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(BittyBuzzFunctionDescriptionTestFixture, BittyBuzzFunctionDescription_addArgument_noSpace) {
    // Given
    // Fulling it
    for (uint i = 0; i < m_functionDescription->getArguments().size(); i++) {
        m_functionDescription->addArgument("arg", FunctionDescriptionArgumentTypeDTO::Int);
    }

    // Then
    bool ret = m_functionDescription->addArgument("arg", FunctionDescriptionArgumentTypeDTO::Int);

    // Expect
    EXPECT_FALSE(ret);
}
