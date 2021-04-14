#include "BittyBuzzVmFixture.h"
#include "BittyBuzzVmTestsUtils.h"
#include "mocks/BSPInterfaceMock.h"
#include "mocks/BittyBuzzClosureRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzMessageServiceInterfaceMock.h"
#include "mocks/BittyBuzzNeighborsManagerInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include "mocks/CircularQueueInterfaceMock.h"
#include <array>
#include <bittybuzz/BittyBuzzClosureRegister.h>
#include <bittybuzz/BittyBuzzLib.h>
#include <bittybuzz/BittyBuzzMessageHandler.h>
#include <bittybuzz/BittyBuzzStringResolver.h>
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <gmock/gmock.h>
#include <register_closure_invalid_args_bytecode.h>
#include <register_closure_invalid_args_string.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_registerClosure_invalidArgs) {
    // Given
    uint16_t boardId = 42;
    std::string functionName = "registeredFunction";

    BittyBuzzClosureRegisterInterfaceMock closureRegister;
    BittyBuzzStringResolverInterfaceMock stringResolver;
    BittyBuzzMessageHandlerInterfaceMock messageHandler;
    BittyBuyzzMessageServiceInterfaceMock messageServiceMock;
    BittyBuzzNeighborsManagerInterfaceMock neighborsManagerMock;

    EXPECT_CALL(neighborsManagerMock, updateNeighbors).Times(0);
    EXPECT_CALL(messageHandler, messageQueueLength).Times(0);
    EXPECT_CALL(closureRegister, registerClosure(testing::_, testing::_, testing::_, testing::_))
        .Times(0);

    EXPECT_CALL(stringResolver, getString(BBZSTRID_registeredFunction))
        .Times(1)
        .WillOnce(testing::Return(functionName.c_str()));

    std::array<BittyBuzzLibMemberRegister, 1> functionRegisters = {
        {{BBZSTRID_register_closure, BittyBuzzUserFunctions::registerClosure}}};
    BittyBuzzLib globalLib(functionRegisters);

    std::vector<std::reference_wrapper<IBittyBuzzLib>> libraries;
    libraries.emplace_back(globalLib);

    SetUp(bcode, bcode_size, boardId, &stringResolver, &messageHandler, &closureRegister,
          &messageServiceMock, &neighborsManagerMock, libraries);

    m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(m_loggerMock->m_logCallCounter, 2);
    EXPECT_EQ(vm->state, BBZVM_STATE_ERROR);
    EXPECT_EQ(vm->error, BBZVM_ERROR_TYPE);
}
