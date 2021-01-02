#include "BittyBuzzVmFixture.h"
#include <swarm_bytecode.h>
#include <bittybuzz/BittyBuzzUserFunctions.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_Swarm_FunctionCalledOnce) {
    // Given
    uint16_t boardId = 1;
    FunctionRegister functionRegister = {BBZSTRID_logInt, bbz_user_functions::logInt};
    SetUp(bcode, bcode_size, boardId, &functionRegister, 1);
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
    FunctionRegister functionRegister = {BBZSTRID_logInt, bbz_user_functions::logInt};
    SetUp(bcode, bcode_size, boardId, &functionRegister, 1);

    // Then
    m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(logCounter, 2);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_Swarm_InTwoSwarm_FunctionCalledQuad) {
    // Given
    uint16_t boardId = 3;
    FunctionRegister functionRegister = {BBZSTRID_logInt, bbz_user_functions::logInt};
    SetUp(bcode, bcode_size, boardId, &functionRegister, 1);

    // Then
    m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(logCounter, 4);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}