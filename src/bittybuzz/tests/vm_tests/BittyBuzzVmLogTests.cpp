#include "BittyBuzzVmFixture.h"
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <gmock/gmock.h>
#include <log_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_log_FunctionCalled) {
    // Given
    uint16_t boardId = 42;

    std::array<FunctionRegister, 1> functionRegister = {
        {{BBZSTRID_log, BittyBuzzUserFunctions::logString}}};

    SetUp(bcode, bcode_size, boardId, functionRegister);

    std::optional<const char*> mockRet = "Hello World";

    EXPECT_CALL(m_bittyBuzzStringResolverMock, getString) // #3
        .Times(1)
        .WillOnce(testing::Return(mockRet.value()));

    // Then
    m_bittybuzzVm->step();

    // Expect
    // Apprend logger prefix string
    std::string expected = std::string("BittyBuzz: ") + std::string(mockRet.value());

    EXPECT_STREQ(m_loggerMock->m_logLastFormat.c_str(), expected.c_str());
    EXPECT_EQ(logCounter, 1);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
