#include "BittyBuzzVmFixture.h"
#include <swarm_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_Swarm_FunctionCalledOnce) {
    // Given
    uint16_t boardId = 1;
    SetUp(bcode, bcode_size, boardId);

    // Then
    m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(logCounter, 1);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_Swarm_FunctionCalledTwice) {
    // Given
    uint16_t boardId = 2;
    SetUp(bcode, bcode_size, boardId);

    // Then
    m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(logCounter, 1);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}