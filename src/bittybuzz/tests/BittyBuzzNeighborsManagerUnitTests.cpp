#include "bbzvm.h"
#include "bittybuzz/BittyBuzzNeighborsManager.h"
#include "mocks/CircularQueueInterfaceMock.h"
#include "mocks/InterlocInterfaceMock.h"
#include <bittybuzz/BittyBuzzLib.h>
#include <gtest/gtest.h>

class BittyBuzzNeighborsManagerTestFixture : public testing::Test {
  protected:
    BittyBuzzNeighborsManager* m_neighborsManager;
    InterlocInterfaceMock m_interlocMock;
    CircularQueueInterfaceMock<uint16_t> m_queueMock;

    const uint16_t m_robotId = 1;

    const char* m_functionName = "functionName";
    bbzvm_t m_bbzvm;

    static void errCallback(bbzvm_error errcode) { (void)errcode; };

    void SetUp() override {
        bbzvm_construct(42);
        bbzvm_set_error_receiver(errCallback);
        vm->error_receiver_fun = errCallback;
        m_neighborsManager = new BittyBuzzNeighborsManager(m_interlocMock, m_queueMock);
    }

    void TearDown() override { delete m_neighborsManager; }
};

TEST_F(BittyBuzzNeighborsManagerTestFixture, BittyBuzzNeighborsManager_updateNeighbors_idInQueue) {
    // Given
    RelativePosition pos;
    pos.m_robotId = m_robotId;
    pos.m_distance = 0;
    pos.m_isInLineOfSight = true;
    pos.m_relativeOrientation = 0;

    const std::optional<std::reference_wrapper<const uint16_t>> robotId = m_robotId;
    const std::optional<RelativePosition> posOpt = pos;
    EXPECT_CALL(m_queueMock, getLength).Times(1).WillOnce(testing::Return(1));
    EXPECT_CALL(m_queueMock, peek).Times(1).WillOnce(testing::Return(robotId));
    EXPECT_CALL(m_queueMock, pop).Times(1);
    EXPECT_CALL(m_interlocMock, getRobotPosition).Times(1).WillOnce(testing::Return(posOpt));

    // Then
    m_neighborsManager->updateNeighbors();

    // Expect
}

TEST_F(BittyBuzzNeighborsManagerTestFixture,
       BittyBuzzNeighborsManager_updateNeighbors_nothingToUpdate) {
    // Given
    EXPECT_CALL(m_queueMock, getLength).Times(1).WillOnce(testing::Return(0));
    EXPECT_CALL(m_queueMock, peek).Times(0);
    EXPECT_CALL(m_queueMock, pop).Times(0);
    EXPECT_CALL(m_interlocMock, getRobotPosition).Times(0);

    // Then
    m_neighborsManager->updateNeighbors();

    // Expect
}
