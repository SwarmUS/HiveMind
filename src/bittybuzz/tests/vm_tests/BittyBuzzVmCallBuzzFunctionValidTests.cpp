#include "BittyBuzzVmFixture.h"
#include "mocks/BittyBuzzClosureRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzMessageServiceInterfaceMock.h"
#include "mocks/BittyBuzzNeighborsManagerInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include <bittybuzz/BittyBuzzLib.h>
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <call_buzz_function_valid_bytecode.h>
#include <call_buzz_function_valid_string.h>
#include <gmock/gmock.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_callBuzzFunction_sendToSelf_valid) {
    // Given
    uint16_t boardId = 42;
    std::string strFunctionName = "buzzFunction";

    BittyBuzzClosureRegisterInterfaceMock closureRegisterMock;
    BittyBuzzStringResolverInterfaceMock stringResolverMock;
    BittyBuzzMessageHandlerInterfaceMock messageHandlerMock;
    BittyBuyzzMessageServiceInterfaceMock messageServiceMock;
    BittyBuzzNeighborsManagerInterfaceMock neighborsManagerMock;

    EXPECT_CALL(neighborsManagerMock, updateNeighbors).Times(1);
    EXPECT_CALL(messageHandlerMock, messageQueueLength).Times(1).WillOnce(testing::Return(0));
    EXPECT_CALL(messageServiceMock, queueBuzzMessages).WillOnce(testing::Return(true));
    EXPECT_CALL(stringResolverMock, getString(BBZSTRID_buzzFunction))
        .WillRepeatedly(testing::Return(strFunctionName.c_str()));

    EXPECT_CALL(messageServiceMock, callBuzzFunction(testing::_, testing::_, testing::_, 2))
        .Times(1);

    std::array<BittyBuzzLibMemberRegister, 1> functionRegisters = {
        {{BBZSTRID_call_buzz_function, BittyBuzzUserFunctions::callBuzzFunction}}};
    BittyBuzzLib globalLib(functionRegisters);

    std::vector<std::reference_wrapper<IBittyBuzzLib>> libraries;
    libraries.emplace_back(globalLib);

    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &closureRegisterMock, &messageServiceMock, &neighborsManagerMock, libraries);

    // Then
    BBVMRet ret = m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(ret, BBVMRet::Ok);
}
