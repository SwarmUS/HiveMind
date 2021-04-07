#include "BittyBuzzVmFixture.h"
#include "BittyBuzzVmTestsUtils.h"
#include "mocks/BittyBuzzClosureRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzMessageServiceInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include "mocks/CircularQueueInterfaceMock.h"
#include "mocks/InterlocInterfaceMock.h"
#include <bittybuzz/BittyBuzzNeighborsManager.h>
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <gmock/gmock.h>
#include <neighbors_distance_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_neighborsBroadcast) {
    // Given
    uint16_t boardId = 42;
    BittyBuzzMessageHandlerInterfaceMock messageHandlerMock;
    BittyBuzzStringResolverInterfaceMock stringResolverMock;
    BittyBuzzClosureRegisterInterfaceMock closureRegisterMock;
    BittyBuyzzMessageServiceInterfaceMock messageServiceMock;
    InterlocInterfaceMock interlocMock;
    CircularQueueInterfaceMock<uint16_t> queueMock;

    BittyBuzzNeighborsManager neighborsManager(interlocMock, queueMock);

    const uint16_t robotId = 1;
    RelativePosition pos;
    pos.m_robotId = robotId;
    pos.m_distance = 42;
    pos.m_isInLineOfSight = true;
    pos.m_relativeOrientation = 24;

    EXPECT_CALL(messageHandlerMock, messageQueueLength).Times(1).WillOnce(testing::Return(0));

    EXPECT_CALL(queueMock, getLength).Times(1).WillOnce(testing::Return(1));
    EXPECT_CALL(queueMock, peek).Times(1).WillOnce(testing::Return(robotId));
    EXPECT_CALL(queueMock, pop).Times(1);
    EXPECT_CALL(interlocMock, getRobotPosition).Times(1).WillOnce(testing::Return(pos));

    std::array<UserFunctionRegister, 1> functionRegister = {
        {{BBZSTRID_assert_true, buzzAssertTrue}}};

    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &closureRegisterMock, &messageServiceMock, &neighborsManager, functionRegister);

    // Then

    m_bittybuzzVm->step();

    // Expect

    EXPECT_EQ(g_assertTrueCallCount, 2);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
