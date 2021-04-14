#include "BittyBuzzVmFixture.h"
#include "mocks/BittyBuzzClosureRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzMessageServiceInterfaceMock.h"
#include "mocks/BittyBuzzNeighborsManagerInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include <bittybuzz/BittyBuzzLib.h>
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <gmock/gmock.h>
#include <log_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_log_FunctionCalled) {
    // Given
    uint16_t boardId = 42;
    BittyBuzzMessageHandlerInterfaceMock messageHandlerMock;
    BittyBuzzStringResolverInterfaceMock stringResolverMock;
    BittyBuzzClosureRegisterInterfaceMock closureRegisterMock;
    BittyBuyzzMessageServiceInterfaceMock messageServiceMock;
    BittyBuzzNeighborsManagerInterfaceMock neighborsManagerMock;

    EXPECT_CALL(neighborsManagerMock, updateNeighbors).Times(1);
    EXPECT_CALL(messageHandlerMock, messageQueueLength).Times(1).WillOnce(testing::Return(0));

    std::array<BittyBuzzUserFunctionRegister, 1> functionRegister = {
        {{BBZSTRID_log, BittyBuzzUserFunctions::log}}};

    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &closureRegisterMock, &messageServiceMock, &neighborsManagerMock, functionRegister);

    std::optional<const char*> mockRet = "Hello World";

    EXPECT_CALL(stringResolverMock, getString).Times(1).WillOnce(testing::Return(mockRet.value()));

    // Then
    m_bittybuzzVm->step();

    // Expect
    // Apprend logger prefix string
    EXPECT_EQ(m_uiMock.m_printCallCounter, 5); // Because we had 4 arguments, 4 call + the first one
    EXPECT_EQ(m_uiMock.m_flushCallCounter, 1);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
