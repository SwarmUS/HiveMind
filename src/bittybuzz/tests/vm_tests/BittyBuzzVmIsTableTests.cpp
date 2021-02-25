#include "BittyBuzzVmFixture.h"
#include "BittyBuzzVmTestsUtils.h"
#include "mocks/BittyBuzzFunctionRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <gmock/gmock.h>
#include <isTable_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_isTable) {
    // Given
    uint16_t boardId = 42;
    BittyBuzzMessageHandlerInterfaceMock messageHandlerMock;
    BittyBuzzStringResolverInterfaceMock stringResolverMock;
    BittyBuzzFunctionRegisterInterfaceMock functionRegisterMock;

    EXPECT_CALL(messageHandlerMock, messageQueueLength).Times(1).WillOnce(testing::Return(0));

    std::array<FunctionRegister, 3> functionRegister = {
        {{BBZSTRID_isTable, BittyBuzzUserFunctions::isTable},
         {BBZSTRID_assertTrue, buzzAssertTrue},
         {BBZSTRID_assertFalse, buzzAssertFalse}}};

    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &functionRegisterMock, functionRegister);

    // Then
    m_bittybuzzVm->step();

    // Expect

    EXPECT_EQ(g_assertTrueCallCount, 1);
    EXPECT_EQ(g_assertFalseCallCount, 6);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
