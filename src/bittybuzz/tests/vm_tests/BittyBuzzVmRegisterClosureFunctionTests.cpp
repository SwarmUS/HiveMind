#include "BittyBuzzVmFixture.h"
#include "BittyBuzzVmTestsUtils.h"
#include "mocks/CircularQueueInterfaceMock.h"
#include <bittybuzz/BittyBuzzClosureRegister.h>
#include <bittybuzz/BittyBuzzMessageHandler.h>
#include <bittybuzz/BittyBuzzStringResolver.h>
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <gmock/gmock.h>
#include <registerClosure_function_bytecode.h>
#include <registerClosure_function_string.h>

TEST_F(BittyBuzzVmTestFixture,
       BittyBuzzVm_integration_registerClosure_registerFunction_callOnMessage) {
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

    FunctionCallRequestDTO fRequest(stringResolver.getString(BBZSTRID_registeredFunction).value(),
                                    NULL, 0);
    UserCallRequestDTO uRequest(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, fRequest);
    RequestDTO request(1, uRequest);
    MessageDTO message(boardId, boardId, request);

    std::array<FunctionRegister, 2> closureRegisters = {
        {{BBZSTRID_assertTrue, buzzAssertTrue},
         {BBZSTRID_registerFunction, BittyBuzzUserFunctions::registerFuntion}}};

    EXPECT_CALL(inputQueueMock, peek).Times(1).WillOnce(testing::Return(message));
    EXPECT_CALL(inputQueueMock, pop).Times(1);
    EXPECT_CALL(inputQueueMock, getLength).Times(1).WillOnce(testing::Return(1));
    EXPECT_CALL(hostOutputQueueMock, push(testing::_)).Times(1);

    SetUp(bcode, bcode_size, boardId, &stringResolver, &messageHandler, &closureRegister,
          closureRegisters);

    // Then
    // Garbage collecting to make sure the pointer is still valid
    bbzvm_gc();
    m_bittybuzzVm->step();

    // Expect

    EXPECT_EQ(g_assertTrueCallCount, 1);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
