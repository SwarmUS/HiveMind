#include "BittyBuzzVmFixture.h"
#include <noStep_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_NoStep_NoError) {
    // Given
    uint16_t boardId = 42;
    SetUp(bcode, bcode_size, boardId, NULL, 0);

    // Then
    m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
