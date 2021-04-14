#include "BittyBuzzVmFixture.h"
#include "BittyBuzzVmTestsUtils.h"
#include "mocks/BittyBuzzClosureRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzMessageServiceInterfaceMock.h"
#include "mocks/BittyBuzzNeighborsManagerInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include <bittybuzz/BittyBuzzLib.h>
#include <bittybuzz/BittyBuzzNeighborsManager.h>
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <gmock/gmock.h>
#include <neighbors_listen_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_neighborsListen) {
    // Given
    uint16_t boardId = 42;
    BittyBuzzMessageHandlerInterfaceMock messageHandlerMock;
    BittyBuzzStringResolverInterfaceMock stringResolverMock;
    BittyBuzzClosureRegisterInterfaceMock closureRegisterMock;
    BittyBuyzzMessageServiceInterfaceMock messageServiceMock;
    BittyBuzzNeighborsManagerInterfaceMock neighborsManagerMock;

    EXPECT_CALL(neighborsManagerMock, updateNeighbors).Times(1);
    EXPECT_CALL(messageHandlerMock, messageQueueLength).Times(1).WillOnce(testing::Return(0));

    uint8_t strId = BBZSTRID_key;
    std::array<uint8_t, 8> payload = {0, 1, 0, strId, 0, 41, 42, 0};
    bbzmsg_payload_t broadcastMessage;
    broadcastMessage.buffer = payload.data();
    broadcastMessage.capacity = 8;
    broadcastMessage.elsize = 1;
    broadcastMessage.datastart = 0;
    broadcastMessage.dataend = 8;

    std::array<BittyBuzzLibMemberRegister, 1> functionRegisters = {
        {{BBZSTRID_assert_true, buzzAssertTrue}}};
    BittyBuzzLib globalLib(0, functionRegisters);

    std::vector<std::reference_wrapper<IBittyBuzzLib>> libraries;
    libraries.emplace_back(globalLib);

    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &closureRegisterMock, &messageServiceMock, &neighborsManagerMock, libraries);

    bbzinmsg_queue_append(&broadcastMessage);

    // Then
    m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(g_assertTrueCallCount, 3);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
