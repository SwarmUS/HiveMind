#include "BittyBuzzVmFixture.h"
#include "mocks/BittyBuzzClosureRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzMessageServiceInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <call_host_function_bytecode.h>
#include <call_host_function_string.h>
#include <gmock/gmock.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_callHostFunction_sendToHost) {
    // Given
    uint16_t boardId = 42;
    std::string strFunctionName = "hostFunction";
    float floatValApprox = bbzfloat_tofloat(bbzfloat_fromfloat(42.24));

    BittyBuzzClosureRegisterInterfaceMock closureRegisterMock;
    BittyBuzzStringResolverInterfaceMock stringResolverMock;
    BittyBuzzMessageHandlerInterfaceMock messageHandlerMock;
    BittyBuyzzMessageServiceInterfaceMock messageServiceMock;

    EXPECT_CALL(messageHandlerMock, messageQueueLength).Times(1).WillOnce(testing::Return(0));
    EXPECT_CALL(stringResolverMock, getString(BBZSTRID_hostFunction))
        .Times(1)
        .WillOnce(testing::Return(strFunctionName.c_str()));

    auto test = [boardId, strFunctionName,
                 floatValApprox](uint16_t hostId, const char* functionName,
                                 const FunctionCallArgumentDTO* args, uint16_t argsLength) {
        EXPECT_EQ(boardId, hostId);
        EXPECT_STREQ(functionName, strFunctionName.c_str());
        EXPECT_EQ(argsLength, 2);
        EXPECT_EQ(std::get<int64_t>(args[0].getArgument()), 42);
        EXPECT_EQ(std::get<float>(args[1].getArgument()), floatValApprox);
        return true;
    };

    EXPECT_CALL(messageServiceMock, callFunction(testing::_, testing::_, testing::_, 2))
        .Times(1)
        .WillOnce(testing::Invoke(test));

    std::array<UserFunctionRegister, 1> functionRegisters = {
        {{BBZSTRID_call_host_function, BittyBuzzUserFunctions::callHostFunction}}};

    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &closureRegisterMock, &messageServiceMock, functionRegisters);

    // Then
    m_bittybuzzVm->step();

    // Expect
}
