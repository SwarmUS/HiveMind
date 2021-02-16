#include "BittyBuzzVmFixture.h"
#include "BittyBuzzVmTestsUtils.h"
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <gmock/gmock.h>
#include <isClosure_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_isClosure) {
    // Given
    uint16_t boardId = 42;

    std::array<FunctionRegister, 3> functionRegister = {
        {{BBZSTRID_isClosure, BittyBuzzUserFunctions::isClosure},
         {BBZSTRID_assertTrue, buzzAssertTrue},
         {BBZSTRID_assertFalse, buzzAssertFalse}}};

    SetUp(bcode, bcode_size, boardId, functionRegister);

    // Then
    m_bittybuzzVm->step();

    // Expect

    EXPECT_EQ(g_assertTrueCallCount, 2);
    EXPECT_EQ(g_assertFalseCallCount, 5);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
