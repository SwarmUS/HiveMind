#include "BittyBuzzVmFixture.h"
#include "BittyBuzzVmTestsUtils.h"
#include "mocks/BittyBuzzFunctionRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <gmock/gmock.h>
#include <registerFunction_bytecode.h>
#include <registerFunction_string.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_registerFunction) {
    // Given
    uint16_t boardId = 42;
    BittyBuzzMessageHandlerInterfaceMock messageHandlerMock;
    BittyBuzzStringResolverInterfaceMock stringResolverMock;
    BittyBuzzFunctionRegisterInterfaceMock functionRegisterMock;

    EXPECT_CALL(messageHandlerMock, messageQueueLength).Times(1).WillOnce(testing::Return(0));

    std::array<FunctionRegister, 2> functionRegister = {
        {{BBZSTRID_assertTrue, buzzAssertTrue},
         {BBZSTRID_registerFunction, BittyBuzzUserFunctions::registerFuntion}}};

    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &functionRegisterMock, functionRegister);

    // Then
    m_bittybuzzVm->step();

    // Expect

    EXPECT_EQ(g_assertTrueCallCount, 1);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
