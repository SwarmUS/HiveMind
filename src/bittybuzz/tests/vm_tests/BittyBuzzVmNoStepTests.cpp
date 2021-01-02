#include "BittyBuzzVmFixture.h"
#include <noStep_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_NoStep_NoError) {
    // Given
    uint16_t boardId = 42;
    std::array<FunctionRegister, 0> functionRegister = {};
    SetUp(bcode, bcode_size, boardId, functionRegister);

    // Then
    m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
