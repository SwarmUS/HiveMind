#include <gtest/gtest.h>
#include <interloc/Interloc.h>
#include <mocks/InterlocManagerInterfaceMock.h>
#include <mocks/LoggerInterfaceMock.h>

class InterlocFixture : public testing::Test {
  protected:
    Interloc* m_interloc;

    InterlocManagerInterfaceMock* m_interlocManagerMock;
    LoggerInterfaceMock* m_loggerMock;

    void SetUp() override {
        m_loggerMock = new LoggerInterfaceMock();
        m_interlocManagerMock = new InterlocManagerInterfaceMock();
        m_interloc = new Interloc(*m_loggerMock, *m_interlocManagerMock);
    }

    void TearDown() override {
        delete m_interloc;
        delete m_interlocManagerMock;
        delete m_loggerMock;
    }
};

TEST_F(InterlocFixture, Interloc_getPosition_robotNotInList) {
    // Given
    uint16_t robotId = 42;

    // Then
    auto ret = m_interloc->getRobotPosition(robotId);

    // Expect
    EXPECT_FALSE(ret.has_value());
}

TEST_F(InterlocFixture, Interloc_getPosition_pushData_robotInList) {
    // Given
    uint16_t robotId = 42;

    InterlocUpdate position;
    position.m_robotId = robotId;
    position.m_isInLineOfSight = true;
    position.m_distance = 42;
    position.m_relativeOrientation = 0;

    // Then
    m_interlocManagerMock->m_positionUpdateCallback(m_interlocManagerMock->m_positionUpdateContext,
                                                    position);
    auto ret = m_interloc->getRobotPosition(robotId);

    // Expect
    EXPECT_TRUE(ret.has_value());
}

TEST_F(InterlocFixture, Interloc_getPosition_validData) {
    // Given
    uint16_t robotId = 42;
    float distance = 42.24;
    float angle = 69.0;
    bool isInLineOfSight = true;

    InterlocUpdate position;
    position.m_robotId = robotId;
    position.m_isInLineOfSight = isInLineOfSight;
    position.m_distance = distance;
    position.m_relativeOrientation = angle;

    // Then
    m_interlocManagerMock->m_positionUpdateCallback(m_interlocManagerMock->m_positionUpdateContext,
                                                    position);
    auto ret = m_interloc->getRobotPosition(robotId);

    // Expect
    EXPECT_EQ(ret->m_distance, distance);
    EXPECT_EQ(ret->m_relativeOrientation, angle);
    EXPECT_EQ(ret->m_isInLineOfSight, isInLineOfSight);
}

TEST_F(InterlocFixture, Interloc_getPosition_updateDistance_validData) {
    // Given
    uint16_t robotId = 42;
    float distance = 42.24;
    float angle = 69.0;
    bool isInLineOfSight = true;

    InterlocUpdate position1;
    position1.m_robotId = robotId;
    position1.m_isInLineOfSight = isInLineOfSight;
    position1.m_distance = distance;
    position1.m_relativeOrientation = angle;

    float updatedDistance = 43;
    InterlocUpdate position2;
    position2.m_robotId = robotId;
    position2.m_distance = updatedDistance;

    // Then
    m_interlocManagerMock->m_positionUpdateCallback(m_interlocManagerMock->m_positionUpdateContext,
                                                    position1);
    m_interlocManagerMock->m_positionUpdateCallback(m_interlocManagerMock->m_positionUpdateContext,
                                                    position2);
    auto ret = m_interloc->getRobotPosition(robotId);

    // Expect
    EXPECT_EQ(ret->m_distance, updatedDistance);
}

TEST_F(InterlocFixture, Interloc_getPosition_updateAngle_validData) {
    // Given
    uint16_t robotId = 42;
    float distance = 42.24;
    float angle = 69.0;
    bool isInLineOfSight = true;

    InterlocUpdate position1;
    position1.m_robotId = robotId;
    position1.m_isInLineOfSight = isInLineOfSight;
    position1.m_distance = distance;
    position1.m_relativeOrientation = angle;

    float updatedAngle = 43;
    InterlocUpdate position2;
    position2.m_robotId = robotId;
    position2.m_relativeOrientation = updatedAngle;

    // Then
    m_interlocManagerMock->m_positionUpdateCallback(m_interlocManagerMock->m_positionUpdateContext,
                                                    position1);
    m_interlocManagerMock->m_positionUpdateCallback(m_interlocManagerMock->m_positionUpdateContext,
                                                    position2);
    auto ret = m_interloc->getRobotPosition(robotId);

    // Expect
    EXPECT_EQ(ret->m_relativeOrientation, updatedAngle);
}

TEST_F(InterlocFixture, Interloc_getPosition_updateLineOfSight_validData) {
    // Given
    uint16_t robotId = 42;
    float distance = 42.24;
    float angle = 69.0;
    bool isInLineOfSight = true;

    InterlocUpdate position1;
    position1.m_robotId = robotId;
    position1.m_isInLineOfSight = isInLineOfSight;
    position1.m_distance = distance;
    position1.m_relativeOrientation = angle;

    bool updatedLOS = false;
    InterlocUpdate position2;
    position2.m_robotId = robotId;
    position2.m_isInLineOfSight = updatedLOS;

    // Then
    m_interlocManagerMock->m_positionUpdateCallback(m_interlocManagerMock->m_positionUpdateContext,
                                                    position1);
    m_interlocManagerMock->m_positionUpdateCallback(m_interlocManagerMock->m_positionUpdateContext,
                                                    position2);
    auto ret = m_interloc->getRobotPosition(robotId);

    // Expect
    EXPECT_EQ(ret->m_isInLineOfSight, updatedLOS);
}

TEST_F(InterlocFixture, Interloc_isLineOfSight_robotNotInList) {
    // Given
    uint16_t robotId = 42;

    bool ret = m_interloc->isLineOfSight(robotId);

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(InterlocFixture, Interloc_isLineOfSight_robotInLOS) {
    // Given
    uint16_t robotId = 42;
    InterlocUpdate positionUpdate;
    positionUpdate.m_robotId = robotId;
    positionUpdate.m_isInLineOfSight = true;

    // Then
    m_interlocManagerMock->m_positionUpdateCallback(m_interlocManagerMock->m_positionUpdateContext,
                                                    positionUpdate);
    bool ret = m_interloc->isLineOfSight(robotId);

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(InterlocFixture, Interloc_isLineOfSight_robotNotInLOS) {
    // Given
    uint16_t robotId = 42;
    InterlocUpdate positionUpdate;
    positionUpdate.m_robotId = robotId;
    positionUpdate.m_isInLineOfSight = false;

    // Then
    m_interlocManagerMock->m_positionUpdateCallback(m_interlocManagerMock->m_positionUpdateContext,
                                                    positionUpdate);
    bool ret = m_interloc->isLineOfSight(robotId);

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(InterlocFixture, Interloc_getPositionsTable_emptyTable) {
    // Then
    auto ret = m_interloc->getPositionsTable();

    // Expect
    EXPECT_EQ(ret.m_positionsLength, 0);
}

TEST_F(InterlocFixture, Interloc_getPositionsTable_elementInTable) {
    uint16_t robotId = 42;
    InterlocUpdate positionUpdate;
    positionUpdate.m_robotId = robotId;
    positionUpdate.m_isInLineOfSight = true;

    // Then
    m_interlocManagerMock->m_positionUpdateCallback(m_interlocManagerMock->m_positionUpdateContext,
                                                    positionUpdate);
    auto ret = m_interloc->getPositionsTable();

    // Expect
    EXPECT_EQ(ret.m_positionsLength, 1);
}

TEST_F(InterlocFixture, Interloc_getPositionsTable_addMoreRobotsThanAllowed) {
    for (int i = 0; i < MAX_ROBOTS_IN_SWARM + 1; i++) {
        InterlocUpdate positionUpdate;
        positionUpdate.m_robotId = i;
        positionUpdate.m_isInLineOfSight = true;

        m_interlocManagerMock->m_positionUpdateCallback(
            m_interlocManagerMock->m_positionUpdateContext, positionUpdate);
    }

    auto ret = m_interloc->getPositionsTable();

    // Expect
    EXPECT_EQ(ret.m_positionsLength, MAX_ROBOTS_IN_SWARM);
}