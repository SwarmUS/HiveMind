#include "BittyBuzzVmFixture.h"
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <logInt_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_logInt_FunctionCalled) {
    // Given
    uint16_t boardId = 42;

    std::array<FunctionRegister, 1> functionRegister = {{{BBZSTRID_logInt, BittyBuzzUserFunctions::logInt}}};
    SetUp(bcode, bcode_size, boardId, functionRegister);

    // Then
    m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(logCounter, 1);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
