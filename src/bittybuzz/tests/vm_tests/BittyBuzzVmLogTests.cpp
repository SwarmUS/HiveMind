#include "BittyBuzzVmFixture.h"
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <log_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_log_FunctionCalled) {
    // Given
    uint16_t boardId = 42;

    std::array<FunctionRegister, 1> functionRegister = {
        {{BBZSTRID_log, BittyBuzzUserFunctions::log}}};

    SetUp(bcode, bcode_size, boardId, functionRegister);

    // Then
    m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(logCounter, 1);
    EXPECT_STREQ(m_loggerMock->m_logLastFormat, "Hello World!");
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
