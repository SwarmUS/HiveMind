#include "BittyBuzzVmFixture.h"
#include "mocks/BittyBuzzClosureRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzMessageServiceInterfaceMock.h"
#include "mocks/BittyBuzzNeighborsManagerInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include <bittybuzz/BittyBuzzLib.h>
#include <undefined_function_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_UndefinedFunction_LogCalledOnError) {
    // Given
    uint16_t boardId = 1;
    BittyBuzzMessageHandlerInterfaceMock messageHandlerMock;
    BittyBuzzStringResolverInterfaceMock stringResolverMock;
    BittyBuzzClosureRegisterInterfaceMock closureRegisterMock;
    BittyBuyzzMessageServiceInterfaceMock messageServiceMock;
    BittyBuzzNeighborsManagerInterfaceMock neighborsManagerMock;

    EXPECT_CALL(neighborsManagerMock, updateNeighbors);
    EXPECT_CALL(messageHandlerMock, messageQueueLength).WillOnce(testing::Return(0));
    EXPECT_CALL(stringResolverMock, getString).WillRepeatedly(testing::Return("Irrelevant"));

    std::vector<std::reference_wrapper<IBittyBuzzLib>> libraries;
    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &closureRegisterMock, &messageServiceMock, &neighborsManagerMock, libraries);

    // Then
    m_bittybuzzVm->step();

    // Expect
    EXPECT_TRUE(logCounter > 0);
    EXPECT_EQ(vm->state, BBZVM_STATE_ERROR);
    EXPECT_EQ(vm->error, BBZVM_ERROR_TYPE);
}
