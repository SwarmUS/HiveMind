#include "BittyBuzzVmFixture.h"
#include "mocks/BittyBuzzFunctionRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include <initAdd_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_AddInt_NoError) {
    // Given
    uint16_t boardId = 42;

    BittyBuzzMessageHandlerInterfaceMock messageHandlerMock;
    BittyBuzzStringResolverInterfaceMock stringResolverMock;
    BittyBuzzFunctionRegisterInterfaceMock functionRegisterMock;

    EXPECT_CALL(messageHandlerMock, messageQueueLength).Times(1).WillOnce(testing::Return(0));

    std::array<FunctionRegister, 0> functionRegister = {};
    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &functionRegisterMock, functionRegister);

    // Then
    m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
