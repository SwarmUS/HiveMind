#include "BittyBuzzVmFixture.h"
#include "BittyBuzzVmTestsUtils.h"
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <gmock/gmock.h>
#include <isInt_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_isInt) {
    // Given
    uint16_t boardId = 42;

    std::array<FunctionRegister, 3> functionRegister = {
        {{BBZSTRID_isInt, BittyBuzzUserFunctions::isInt},
         {BBZSTRID_assertTrue, buzzAssertTrue},
         {BBZSTRID_assertFalse, buzzAssertFalse}}};

    SetUp(bcode, bcode_size, boardId, functionRegister);

    // Then
    m_bittybuzzVm->step();

    // Expect

    EXPECT_EQ(g_assertTrueCallCount, 1);
    EXPECT_EQ(g_assertFalseCallCount, 6);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
