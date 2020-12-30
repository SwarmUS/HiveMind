#include "BittyBuzzVmFixture.h"
#include <logInt_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_logInt_FunctionCalled) {
    // Given
    uint16_t boardId = 42;
    SetUp(bcode, bcode_size, boardId);

    // Then
    m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(logCounter, 1);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
