#include "BittyBuzzVmFixture.h"
#include "BittyBuzzVmTestsUtils.h"
#include "mocks/BittyBuzzClosureRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include "mocks/CircularQueueInterfaceMock.h"
#include <array>
#include <bittybuzz/BittyBuzzClosureRegister.h>
#include <bittybuzz/BittyBuzzMessageHandler.h>
#include <bittybuzz/BittyBuzzStringResolver.h>
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <gmock/gmock.h>
#include <register_closure_lambda_bytecode.h>
#include <register_closure_lambda_string.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_registerClosure_registerLambda) {
    // Given
    uint16_t boardId = 42;
    std::string functionName = "registeredFunction";

    BittyBuzzClosureRegisterInterfaceMock closureRegister;
    BittyBuzzStringResolverInterfaceMock stringResolver;
    BittyBuzzMessageHandlerInterfaceMock messageHandler;

    EXPECT_CALL(messageHandler, messageQueueLength).Times(1).WillOnce(testing::Return(0));
    EXPECT_CALL(stringResolver, getString).Times(1).WillOnce(testing::Return(functionName.c_str()));
    EXPECT_CALL(closureRegister, registerClosure(functionName.c_str(), testing::_))
        .Times(1)
        .WillOnce(testing::Return(true));

    std::array<UserFunctionRegister, 2> functionRegisters = {
        {{BBZSTRID_assert_true, buzzAssertTrue},
         {BBZSTRID_register_closure, BittyBuzzUserFunctions::registerClosure}}};

    SetUp(bcode, bcode_size, boardId, &stringResolver, &messageHandler, &closureRegister,
          functionRegisters);

    // Then
    // Garbage collecting to make sure the pointer is still valid
    bbzvm_gc();
    m_bittybuzzVm->step();

    // Expect
}

TEST_F(BittyBuzzVmTestFixture,
       BittyBuzzVm_integration_registerClosure_registerLambda_callOnMessage) {
    // Given
    uint16_t boardId = 42;
    CircularQueueInterfaceMock<MessageDTO> inputQueueMock;
    CircularQueueInterfaceMock<MessageDTO> hostOutputQueueMock;
    CircularQueueInterfaceMock<MessageDTO> remoteOutputQueueMock;

    BittyBuzzClosureRegister closureRegister;
    BittyBuzzStringResolver stringResolver(g_bbzStringResolverArray.data(),
                                           g_bbzStringResolverArray.size(), BBZSTRING_OFFSET,
                                           *m_loggerMock);

    BittyBuzzMessageHandler messageHandler(closureRegister, inputQueueMock, hostOutputQueueMock,
                                           remoteOutputQueueMock, boardId, *m_loggerMock);

    std::array<FunctionCallArgumentDTO, 1> fArgs = {{{(int64_t)42}}};
    FunctionCallRequestDTO fRequest(stringResolver.getString(BBZSTRID_registeredLambda).value(),
                                    fArgs.data(), fArgs.size());
    UserCallRequestDTO uRequest(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, fRequest);
    RequestDTO request(1, uRequest);
    MessageDTO message(boardId, boardId, request);

    std::array<UserFunctionRegister, 2> functionRegisters = {
        {{BBZSTRID_assert_true, buzzAssertTrue},
         {BBZSTRID_register_closure, BittyBuzzUserFunctions::registerClosure}}};

    EXPECT_CALL(inputQueueMock, peek).Times(1).WillOnce(testing::Return(message));
    EXPECT_CALL(inputQueueMock, pop).Times(1);
    EXPECT_CALL(inputQueueMock, getLength).Times(1).WillOnce(testing::Return(1));
    EXPECT_CALL(hostOutputQueueMock, push(testing::_)).Times(1);

    SetUp(bcode, bcode_size, boardId, &stringResolver, &messageHandler, &closureRegister,
          functionRegisters);

    // Then
    // Garbage collecting to make sure the pointer is still valid
    bbzvm_gc();
    m_bittybuzzVm->step();

    // Expect

    EXPECT_EQ(g_assertTrueCallCount, 1);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
