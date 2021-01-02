#include "BittyBuzzVmFixture.h"
#include <logInt_bytecode.h>
#include <bittybuzz/BittyBuzzUserFunctions.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_logInt_FunctionCalled) {
    // Given
    uint16_t boardId = 42;

    FunctionRegister functionRegister = {BBZSTRID_logInt, bbz_user_functions::logInt};
    SetUp(bcode, bcode_size, boardId, &functionRegister, 1);

    // Then
    m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(logCounter, 1);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
