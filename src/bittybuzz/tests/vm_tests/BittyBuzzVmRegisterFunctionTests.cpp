#include "BittyBuzzVmFixture.h"
#include "BittyBuzzVmTestsUtils.h"
#include "mocks/CircularQueueInterfaceMock.h"
#include <bittybuzz/BittyBuzzFunctionRegister.h>
#include <bittybuzz/BittyBuzzMessageHandler.h>
#include <bittybuzz/BittyBuzzStringResolver.h>
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <gmock/gmock.h>
#include <registerFunction_bytecode.h>
#include <registerFunction_string.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_integration_registerFunction_callOnMessage) {
    // Given
    uint16_t boardId = 42;
    CircularQueueInterfaceMock<MessageDTO> inputQueueMock;
    CircularQueueInterfaceMock<MessageDTO> hostOutputQueueMock;
    CircularQueueInterfaceMock<MessageDTO> remoteOutputQueueMock;

    BittyBuzzFunctionRegister functionRegister;
    BittyBuzzStringResolver stringResolver(g_bbzStringResolverArray.data(),
                                           g_bbzStringResolverArray.size(), BBZSTRING_OFFSET,
                                           *m_loggerMock);

    BittyBuzzMessageHandler messageHandler(functionRegister, inputQueueMock, hostOutputQueueMock,
                                           remoteOutputQueueMock, boardId, *m_loggerMock);

    FunctionCallRequestDTO fRequest(stringResolver.getString(BBZSTRID_registeredFunction).value(),
                                    NULL, 0);
    UserCallRequestDTO uRequest(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, fRequest);
    RequestDTO request(1, uRequest);
    MessageDTO message(boardId, boardId, request);

    std::array<FunctionRegister, 2> functionRegisters = {
        {{BBZSTRID_assertTrue, buzzAssertTrue},
         {BBZSTRID_registerFunction, BittyBuzzUserFunctions::registerFuntion}}};

    SetUp(bcode, bcode_size, boardId, &stringResolver, &messageHandler, &functionRegister,
          functionRegisters);

    // Then
    m_bittybuzzVm->step();

    // Expect

    EXPECT_EQ(g_assertTrueCallCount, 1);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
