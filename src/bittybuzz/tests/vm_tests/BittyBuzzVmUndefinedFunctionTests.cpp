#include "BittyBuzzVmFixture.h"
#include <undefinedFunction_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_UndefinedFunction_LogCalledOnError) {
    // Given
    uint16_t boardId = 1;
    SetUp(bcode, bcode_size, boardId, NULL, 0);

    // Then
    m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(logCounter, 1);
    EXPECT_EQ(vm->state, BBZVM_STATE_ERROR);
    EXPECT_EQ(vm->error, BBZVM_ERROR_TYPE);
}