#include "BittyBuzzVmFixture.h"
#include "mocks/BittyBuzzClosureRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <gmock/gmock.h>
#include <log_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_log_FunctionCalled) {
    // Given
    uint16_t boardId = 42;
    BittyBuzzMessageHandlerInterfaceMock messageHandlerMock;
    BittyBuzzStringResolverInterfaceMock stringResolverMock;
    BittyBuzzClosureRegisterInterfaceMock closureRegisterMock;

    EXPECT_CALL(messageHandlerMock, messageQueueLength).Times(1).WillOnce(testing::Return(0));

    std::array<UserFunctionRegister, 1> functionRegister = {
        {{BBZSTRID_log, BittyBuzzUserFunctions::logString}}};

    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &closureRegisterMock, functionRegister);

    std::optional<const char*> mockRet = "Hello World";

    EXPECT_CALL(stringResolverMock, getString).Times(1).WillOnce(testing::Return(mockRet.value()));

    // Then
    m_bittybuzzVm->step();

    // Expect
    // Apprend logger prefix string
    std::string expected = std::string("BBZ: ") + std::string(mockRet.value());

    EXPECT_STREQ(m_loggerMock->m_logLastFormat.c_str(), expected.c_str());
    EXPECT_EQ(logCounter, 1);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
