#include "BittyBuzzVmFixture.h"
#include "BittyBuzzVmTestsUtils.h"
#include "mocks/BittyBuzzClosureRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzMessageServiceInterfaceMock.h"
#include "mocks/BittyBuzzNeighborsManagerInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include <bittybuzz/BittyBuzzLib.h>
#include <gtest/gtest.h>
#include <stigmergy_put_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_Stigmergy_put_pushSuccess) {
    // Given
    uint16_t boardId = 1;
    BittyBuzzMessageHandlerInterfaceMock messageHandlerMock;
    BittyBuzzStringResolverInterfaceMock stringResolverMock;
    BittyBuzzClosureRegisterInterfaceMock closureRegisterMock;
    BittyBuyzzMessageServiceInterfaceMock messageServiceMock;
    BittyBuzzNeighborsManagerInterfaceMock neighborsManagerMock;

    EXPECT_CALL(neighborsManagerMock, updateNeighbors).Times(1);
    EXPECT_CALL(messageHandlerMock, messageQueueLength).Times(1).WillOnce(testing::Return(0));
    EXPECT_CALL(messageServiceMock, sendBuzzMessage).Times(1).WillOnce(testing::Return(true));

    std::array<BittyBuzzUserFunctionRegister, 0> functionRegister = {};
    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &closureRegisterMock, &messageServiceMock, &neighborsManagerMock, functionRegister);
    // Then
    bool ret = m_bittybuzzVm->step();

    // Expect
    EXPECT_TRUE(ret);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
    EXPECT_EQ(bbzoutmsg_queue_size(), 0);
}

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_Stigmergy_put_pushFail) {
    // Given
    uint16_t boardId = 1;
    BittyBuzzMessageHandlerInterfaceMock messageHandlerMock;
    BittyBuzzStringResolverInterfaceMock stringResolverMock;
    BittyBuzzClosureRegisterInterfaceMock closureRegisterMock;
    BittyBuyzzMessageServiceInterfaceMock messageServiceMock;
    BittyBuzzNeighborsManagerInterfaceMock neighborsManagerMock;

    EXPECT_CALL(neighborsManagerMock, updateNeighbors).Times(1);
    EXPECT_CALL(messageHandlerMock, messageQueueLength).Times(1).WillOnce(testing::Return(0));
    EXPECT_CALL(messageServiceMock, sendBuzzMessage).Times(1).WillOnce(testing::Return(false));

    std::array<BittyBuzzUserFunctionRegister, 0> functionRegister = {};
    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &closureRegisterMock, &messageServiceMock, &neighborsManagerMock, functionRegister);
    // Then
    bool ret = m_bittybuzzVm->step();

    // Expect
    EXPECT_FALSE(ret);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
    EXPECT_EQ(bbzoutmsg_queue_size(), 1);
}
