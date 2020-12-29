#include "BittyBuzzVmFixture.h"
#include <initAdd_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_AddInt_NoError) {
    // Given
    uint16_t boardId = 42;
    SetUp(bcode, bcode_size, boardId);

    // Then
    m_bittybuzzVm->step();

    //Expect
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
