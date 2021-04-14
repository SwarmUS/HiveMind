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
#include <bittybuzz/BittyBuzzMessageHandler.h>
#include <bittybuzz/BittyBuzzStringResolver.h>
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <gmock/gmock.h>
#include <register_closure_function_self_bytecode.h>
#include <register_closure_function_self_string.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_registerClosure_registerFunction_self) {
    // Given
    uint16_t boardId = 42;
    std::string functionName = "registeredFunction";
    std::string argIntName = "arg_int";
    std::string argFloatName = "arg_float";

    BittyBuzzClosureRegisterInterfaceMock closureRegister;
    BittyBuzzStringResolverInterfaceMock stringResolver;
    BittyBuzzMessageHandlerInterfaceMock messageHandler;
    BittyBuyzzMessageServiceInterfaceMock messageServiceMock;
    BittyBuzzNeighborsManagerInterfaceMock neighborsManagerMock;

    EXPECT_CALL(neighborsManagerMock, updateNeighbors).Times(1);
    EXPECT_CALL(messageHandler, messageQueueLength).Times(1).WillOnce(testing::Return(0));
    EXPECT_CALL(closureRegister,
                registerClosure(functionName.c_str(), testing::_, testing::_, testing::_))
        .Times(1)
        .WillOnce(testing::Return(true));

    EXPECT_CALL(stringResolver, getString(BBZSTRID_registeredFunction))
        .Times(1)
        .WillOnce(testing::Return(functionName.c_str()));
    EXPECT_CALL(stringResolver, getString(BBZSTRID_arg_int))
        .Times(1)
        .WillOnce(testing::Return(argIntName.c_str()));
    EXPECT_CALL(stringResolver, getString(BBZSTRID_arg_float))
        .Times(1)
        .WillOnce(testing::Return(argFloatName.c_str()));

    std::array<BittyBuzzUserFunctionRegister, 2> functionRegisters = {
        {{BBZSTRID_assert_true, buzzAssertTrue},
         {BBZSTRID_register_closure, BittyBuzzUserFunctions::registerClosure}}};

    SetUp(bcode, bcode_size, boardId, &stringResolver, &messageHandler, &closureRegister,
          &messageServiceMock, &neighborsManagerMock, functionRegisters);

    // Then
    // Garbage collecting to make sure the pointer is still valid
    bbzvm_gc();
    m_bittybuzzVm->step();

    // Expect
}

TEST_F(BittyBuzzVmTestFixture,
       BittyBuzzVm_integration_registerClosure_registerFunction_self_callOnMessage) {
    // Given
    uint16_t boardId = 42;
    BSPInterfaceMock bspMock(boardId);
    CircularQueueInterfaceMock<MessageDTO> inputQueueMock;
    CircularQueueInterfaceMock<MessageDTO> hostOutputQueueMock;
    CircularQueueInterfaceMock<MessageDTO> remoteOutputQueueMock;

    BittyBuzzClosureRegister closureRegister;
    BittyBuzzStringResolver stringResolver(g_bbzStringResolverArray.data(),
                                           g_bbzStringResolverArray.size(), BBZSTRING_OFFSET,
                                           *m_loggerMock);

    BittyBuzzMessageHandler messageHandler(closureRegister, inputQueueMock, hostOutputQueueMock,
                                           remoteOutputQueueMock, bspMock, *m_loggerMock);
    BittyBuyzzMessageServiceInterfaceMock messageServiceMock;
    BittyBuzzNeighborsManagerInterfaceMock neighborsManagerMock;

    std::array<FunctionCallArgumentDTO, 2> fArgs = {{{(int64_t)42}, {(float)42.24}}};
    FunctionCallRequestDTO fRequest(stringResolver.getString(BBZSTRID_registeredFunction).value(),
                                    fArgs.data(), fArgs.size());
    UserCallRequestDTO uRequest(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, fRequest);
    RequestDTO request(1, uRequest);
    MessageDTO message(boardId, boardId, request);

    std::array<BittyBuzzUserFunctionRegister, 2> functionRegisters = {
        {{BBZSTRID_assert_true, buzzAssertTrue},
         {BBZSTRID_register_closure, BittyBuzzUserFunctions::registerClosure}}};

    EXPECT_CALL(neighborsManagerMock, updateNeighbors).Times(1);
    EXPECT_CALL(inputQueueMock, peek).Times(1).WillOnce(testing::Return(message));
    EXPECT_CALL(inputQueueMock, pop).Times(1);
    EXPECT_CALL(inputQueueMock, getLength).Times(1).WillOnce(testing::Return(1));
    EXPECT_CALL(hostOutputQueueMock, push(testing::_)).Times(1);

    SetUp(bcode, bcode_size, boardId, &stringResolver, &messageHandler, &closureRegister,
          &messageServiceMock, &neighborsManagerMock, functionRegisters);

    // Then
    // Garbage collecting to make sure the pointer is still valid
    bbzvm_gc();
    m_bittybuzzVm->step();

    // Expect

    EXPECT_EQ(g_assertTrueCallCount, 3);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
