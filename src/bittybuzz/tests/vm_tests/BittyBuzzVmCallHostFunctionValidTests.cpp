#include "BittyBuzzVmFixture.h"
#include "mocks/BittyBuzzClosureRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzMessageServiceInterfaceMock.h"
#include "mocks/BittyBuzzNeighborsManagerInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include <bittybuzz/BittyBuzzLib.h>
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <call_host_function_valid_bytecode.h>
#include <call_host_function_valid_string.h>
#include <gmock/gmock.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_callHostFunction_sendToHost_valid) {
    // Given
    uint16_t boardId = 42;
    std::string strFunctionName = "hostFunction";
    // Buzz has an error since it's floats are on 16 bits
    float floatValApprox = bbzfloat_tofloat(bbzfloat_fromfloat(42.24));

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

    EXPECT_CALL(messageServiceMock, callHostFunction(testing::_, testing::_, testing::_, 2))
        .Times(1)
        .WillOnce(testing::Invoke(test));

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
}
