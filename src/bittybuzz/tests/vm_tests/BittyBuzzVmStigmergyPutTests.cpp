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
    EXPECT_CALL(messageHandlerMock, messageQueueLength).WillOnce(testing::Return(0));
    EXPECT_CALL(messageServiceMock, queueBuzzMessages).WillOnce(testing::Return(true));

    std::vector<std::reference_wrapper<IBittyBuzzLib>> libraries;
    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &closureRegisterMock, &messageServiceMock, &neighborsManagerMock, libraries);
    // Then
    BBVMRet ret = m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(ret, BBVMRet::Ok);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
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
    EXPECT_CALL(messageHandlerMock, messageQueueLength).WillOnce(testing::Return(0));
    EXPECT_CALL(messageServiceMock, queueBuzzMessages).WillOnce(testing::Return(false));

    std::vector<std::reference_wrapper<IBittyBuzzLib>> libraries;
    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &closureRegisterMock, &messageServiceMock, &neighborsManagerMock, libraries);
    // Then
    BBVMRet ret = m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(ret, BBVMRet::OutMsgErr);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
