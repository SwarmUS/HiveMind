#include "BittyBuzzVmFixture.h"
#include "mocks/BittyBuzzClosureRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzMessageServiceInterfaceMock.h"
#include "mocks/BittyBuzzNeighborsManagerInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include <bittybuzz/BittyBuzzLib.h>
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <call_host_function_invalidKeys_bytecode.h>
#include <call_host_function_invalidKeys_string.h>
#include <gmock/gmock.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_callHostFunction_sendToHost_invalidKeys) {
    // Given
    uint16_t boardId = 42;
    std::string strFunctionName = "hostFunction";

    BittyBuzzClosureRegisterInterfaceMock closureRegisterMock;
    BittyBuzzStringResolverInterfaceMock stringResolverMock;
    BittyBuzzMessageHandlerInterfaceMock messageHandlerMock;
    BittyBuyzzMessageServiceInterfaceMock messageServiceMock;
    BittyBuzzNeighborsManagerInterfaceMock neighborsManagerMock;

    EXPECT_CALL(neighborsManagerMock, updateNeighbors).Times(1);
    EXPECT_CALL(messageHandlerMock, messageQueueLength).Times(1).WillOnce(testing::Return(0));
    EXPECT_CALL(stringResolverMock, getString(BBZSTRID_hostFunction))
        .Times(1)
        .WillOnce(testing::Return(strFunctionName.c_str()));

    EXPECT_CALL(messageServiceMock, callHostFunction(testing::_, testing::_, testing::_, 2))
        .Times(0);

    std::array<BittyBuzzLibMemberRegister, 1> functionRegisters = {
        {{BBZSTRID_call_host_function, BittyBuzzUserFunctions::callHostFunction}}};
    BittyBuzzLib globalLib(functionRegisters);

    std::vector<std::reference_wrapper<IBittyBuzzLib>> libraries;
    libraries.emplace_back(globalLib);

    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &closureRegisterMock, &messageServiceMock, &neighborsManagerMock, libraries);

    // Then
    m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(m_loggerMock->m_logCallCounter, 2);
    EXPECT_EQ(vm->state, BBZVM_STATE_ERROR);
    EXPECT_EQ(vm->error, BBZVM_ERROR_TYPE);
}
