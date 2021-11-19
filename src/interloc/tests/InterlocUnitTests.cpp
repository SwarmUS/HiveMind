#include <gtest/gtest.h>
#include <interloc/Interloc.h>
#include <mocks/CircularQueueInterfaceMock.h>
#include <mocks/InterlocManagerInterfaceMock.h>
#include <mocks/InterlocMessageHandlerMock.h>
#include <mocks/LoggerInterfaceMock.h>
#include <mocks/NotificationQueueInterfaceMock.h>

class InterlocFixture : public testing::Test {
  protected:
    Interloc* m_interloc;

    InterlocManagerInterfaceMock* m_interlocManagerMock;
    LoggerInterfaceMock* m_loggerMock;
    CircularQueueInterfaceMock<uint16_t> m_updateOutputQueue;
    NotificationQueueInterfaceMock<InterlocUpdate> m_updateInputQueue;
    InterlocMessageHandlerMock m_messageHandlerMock;

    void SetUp() override {
        m_loggerMock = new LoggerInterfaceMock();
        m_interlocManagerMock = new InterlocManagerInterfaceMock();
        m_interloc = new Interloc(*m_loggerMock, *m_interlocManagerMock, m_messageHandlerMock,
                                  m_updateOutputQueue, m_updateInputQueue);
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

TEST_F(InterlocFixture, Interloc_getPosition_process_emptyListDoesNotPop) {
    // GIVEN
    EXPECT_CALL(m_updateInputQueue, isEmpty).Times(1).WillOnce(testing::Return(true));
    EXPECT_CALL(m_updateInputQueue, wait(testing::_)).Times(1);
    EXPECT_CALL(m_updateInputQueue, peek)
        .Times(1)
        .WillOnce(testing::Return(std::optional<InterlocUpdate>()));

    // EXPECT
    EXPECT_CALL(m_updateInputQueue, pop).Times(0);
    EXPECT_CALL(m_updateOutputQueue, push(testing::_)).Times(0);

    m_interloc->process();
}

TEST_F(InterlocFixture, Interloc_getPosition_pushData_robotInList) {
    // Given
    uint16_t robotId = 42;

    InterlocUpdate position;
    position.m_robotId = robotId;
    position.m_isInLineOfSight = true;
    position.m_distance = 42;
    position.m_angleOfArrival = 0;

    uint16_t queuePushValue;
    EXPECT_CALL(m_updateOutputQueue, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&queuePushValue)));

    EXPECT_CALL(m_updateInputQueue, isEmpty).Times(1).WillOnce(testing::Return(false));
    EXPECT_CALL(m_updateInputQueue, peek).Times(1).WillOnce(testing::Return(position));
    EXPECT_CALL(m_updateInputQueue, pop).Times(1);

    // Then
    m_interloc->process();
    auto ret = m_interloc->getRobotPosition(robotId);

    // Expect
    EXPECT_TRUE(ret.has_value());
    EXPECT_EQ(queuePushValue, robotId);
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
    position.m_angleOfArrival = angle;

    uint16_t queuePushValue;
    EXPECT_CALL(m_updateOutputQueue, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&queuePushValue)));

    EXPECT_CALL(m_updateInputQueue, isEmpty).Times(1).WillOnce(testing::Return(false));
    EXPECT_CALL(m_updateInputQueue, peek).Times(1).WillOnce(testing::Return(position));
    EXPECT_CALL(m_updateInputQueue, pop).Times(1);

    // Then
    m_interloc->process();
    auto ret = m_interloc->getRobotPosition(robotId);

    // Expect
    EXPECT_NE(ret->m_distance, 0);
    EXPECT_NE(ret->m_angle, 0);
    EXPECT_EQ(ret->m_isInLineOfSight, isInLineOfSight);
    EXPECT_EQ(queuePushValue, robotId);
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
    position1.m_angleOfArrival = angle;

    float updatedDistance = 43;
    InterlocUpdate position2;
    position2.m_robotId = robotId;
    position2.m_distance = updatedDistance;

    uint16_t queuePushValue1;
    uint16_t queuePushValue2;
    EXPECT_CALL(m_updateOutputQueue, push(testing::_))
        .Times(2)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&queuePushValue1)))
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&queuePushValue2)));

    EXPECT_CALL(m_updateInputQueue, isEmpty).Times(2).WillRepeatedly(testing::Return(false));
    EXPECT_CALL(m_updateInputQueue, peek)
        .Times(2)
        .WillOnce(testing::Return(position1))
        .WillOnce(testing::Return(position2));
    EXPECT_CALL(m_updateInputQueue, pop).Times(2);

    // Then
    m_interloc->process();
    m_interloc->process();
    auto ret = m_interloc->getRobotPosition(robotId);

    // Expect
    EXPECT_NE(ret->m_distance, distance);
    EXPECT_EQ(queuePushValue1, robotId);
    EXPECT_EQ(queuePushValue2, robotId);
}

TEST_F(InterlocFixture, Interloc_getPosition_updateOrientation_validData) {
    // Given
    uint16_t robotId = 42;
    float distance = 42.24;
    float angle = 69.0;
    bool isInLineOfSight = true;

    InterlocUpdate position1;
    position1.m_robotId = robotId;
    position1.m_isInLineOfSight = isInLineOfSight;
    position1.m_distance = distance;
    position1.m_angleOfArrival = angle;

    float updatedAngle = 43;
    InterlocUpdate position2;
    position2.m_robotId = robotId;
    position2.m_angleOfArrival = updatedAngle;

    uint16_t queuePushValue1;
    uint16_t queuePushValue2;
    EXPECT_CALL(m_updateOutputQueue, push(testing::_))
        .Times(2)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&queuePushValue1)))
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&queuePushValue2)));

    EXPECT_CALL(m_updateInputQueue, isEmpty).Times(2).WillRepeatedly(testing::Return(false));
    EXPECT_CALL(m_updateInputQueue, peek)
        .Times(2)
        .WillOnce(testing::Return(position1))
        .WillOnce(testing::Return(position2));
    EXPECT_CALL(m_updateInputQueue, pop).Times(2);

    // Then
    m_interloc->process();
    m_interloc->process();
    auto ret = m_interloc->getRobotPosition(robotId);

    // Expect
    EXPECT_NE(ret->m_angle, angle);
    EXPECT_EQ(queuePushValue1, robotId);
    EXPECT_EQ(queuePushValue2, robotId);
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
    position1.m_angleOfArrival = angle;

    float updatedAngle = 43;
    InterlocUpdate position2;
    position2.m_robotId = robotId;
    position2.m_angleOfArrival = updatedAngle;

    uint16_t queuePushValue1;
    uint16_t queuePushValue2;
    EXPECT_CALL(m_updateOutputQueue, push(testing::_))
        .Times(2)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&queuePushValue1)))
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&queuePushValue2)));

    EXPECT_CALL(m_updateInputQueue, isEmpty).Times(2).WillRepeatedly(testing::Return(false));
    EXPECT_CALL(m_updateInputQueue, peek)
        .Times(2)
        .WillOnce(testing::Return(position1))
        .WillOnce(testing::Return(position2));
    EXPECT_CALL(m_updateInputQueue, pop).Times(2);

    // Then
    m_interloc->process();
    m_interloc->process();
    auto ret = m_interloc->getRobotPosition(robotId);

    // Expect
    EXPECT_NE(ret->m_angle, angle);
    EXPECT_EQ(queuePushValue1, robotId);
    EXPECT_EQ(queuePushValue2, robotId);
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
    position1.m_angleOfArrival = angle;

    bool updatedLOS = false;
    InterlocUpdate position2;
    position2.m_robotId = robotId;
    position2.m_isInLineOfSight = updatedLOS;

    uint16_t queuePushValue1;
    uint16_t queuePushValue2;
    EXPECT_CALL(m_updateOutputQueue, push(testing::_))
        .Times(2)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&queuePushValue1)))
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&queuePushValue2)));

    EXPECT_CALL(m_updateInputQueue, isEmpty).Times(2).WillRepeatedly(testing::Return(false));
    EXPECT_CALL(m_updateInputQueue, peek)
        .Times(2)
        .WillOnce(testing::Return(position1))
        .WillOnce(testing::Return(position2));
    EXPECT_CALL(m_updateInputQueue, pop).Times(2);

    // Then
    m_interloc->process();
    m_interloc->process();
    auto ret = m_interloc->getRobotPosition(robotId);

    // Expect
    EXPECT_EQ(ret->m_isInLineOfSight, updatedLOS);
    EXPECT_EQ(queuePushValue1, robotId);
    EXPECT_EQ(queuePushValue2, robotId);
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

    uint16_t queuePushValue;
    EXPECT_CALL(m_updateOutputQueue, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&queuePushValue)));

    EXPECT_CALL(m_updateInputQueue, isEmpty).Times(1).WillRepeatedly(testing::Return(false));
    EXPECT_CALL(m_updateInputQueue, peek).Times(1).WillOnce(testing::Return(positionUpdate));
    EXPECT_CALL(m_updateInputQueue, pop).Times(1);

    // Then
    m_interloc->process();
    bool ret = m_interloc->isLineOfSight(robotId);

    // Expect
    EXPECT_TRUE(ret);
    EXPECT_EQ(queuePushValue, robotId);
}

TEST_F(InterlocFixture, Interloc_isLineOfSight_robotNotInLOS) {
    // Given
    uint16_t robotId = 42;
    InterlocUpdate positionUpdate;
    positionUpdate.m_robotId = robotId;
    positionUpdate.m_isInLineOfSight = false;

    uint16_t queuePushValue;
    EXPECT_CALL(m_updateOutputQueue, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&queuePushValue)));

    EXPECT_CALL(m_updateInputQueue, isEmpty).Times(1).WillRepeatedly(testing::Return(false));
    EXPECT_CALL(m_updateInputQueue, peek).Times(1).WillOnce(testing::Return(positionUpdate));
    EXPECT_CALL(m_updateInputQueue, pop).Times(1);

    // Then
    m_interloc->process();
    bool ret = m_interloc->isLineOfSight(robotId);

    // Expect
    EXPECT_FALSE(ret);
    EXPECT_EQ(queuePushValue, robotId);
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

    EXPECT_CALL(m_updateOutputQueue, push(testing::_));

    EXPECT_CALL(m_updateInputQueue, isEmpty).Times(1).WillRepeatedly(testing::Return(false));
    EXPECT_CALL(m_updateInputQueue, peek).Times(1).WillOnce(testing::Return(positionUpdate));
    EXPECT_CALL(m_updateInputQueue, pop).Times(1);

    // Then
    m_interloc->process();
    auto ret = m_interloc->getPositionsTable();

    // Expect
    EXPECT_EQ(ret.m_positionsLength, 1);
}

TEST_F(InterlocFixture, Interloc_getPositionsTable_addMoreRobotsThanAllowed) {
    EXPECT_CALL(m_updateOutputQueue, push(testing::_)).Times(MAX_AGENTS_IN_SWARM);

    InterlocUpdate positionUpdate;
    positionUpdate.m_isInLineOfSight = true;
    int timesCalled = 0;

    EXPECT_CALL(m_updateInputQueue, isEmpty)
        .Times(MAX_AGENTS_IN_SWARM + 1)
        .WillRepeatedly(testing::Return(false));
    EXPECT_CALL(m_updateInputQueue, peek)
        .Times(MAX_AGENTS_IN_SWARM + 1)
        .WillRepeatedly(testing::DoAll(testing::Invoke([&timesCalled, &positionUpdate]() {
                                           timesCalled++;
                                           positionUpdate.m_robotId = timesCalled;
                                       }),
                                       testing::ReturnPointee(&positionUpdate)));
    EXPECT_CALL(m_updateInputQueue, pop).Times(MAX_AGENTS_IN_SWARM + 1);

    for (int i = 0; i < MAX_AGENTS_IN_SWARM + 1; i++) {
        m_interloc->process();
    }

    auto ret = m_interloc->getPositionsTable();

    // Expect
    EXPECT_EQ(ret.m_positionsLength, MAX_AGENTS_IN_SWARM);
}

TEST_F(InterlocFixture, Interloc_addUpdate_dumpToHost) {
    EXPECT_CALL(m_messageHandlerMock, getDumpEnabled).WillOnce(testing::Return(true));
    EXPECT_CALL(m_updateOutputQueue, push(testing::_)).Times(InterlocDumpDTO::MAX_UPDATES_SIZE);

    InterlocUpdate positionUpdate;
    positionUpdate.m_robotId = 1;
    positionUpdate.m_isInLineOfSight = true;

    int timesCalled = 0;

    EXPECT_CALL(m_updateInputQueue, isEmpty).WillRepeatedly(testing::Return(false));
    EXPECT_CALL(m_updateInputQueue, peek)
        .WillRepeatedly(testing::DoAll(testing::Invoke([&timesCalled, &positionUpdate]() {
                                           positionUpdate.m_distance = timesCalled;
                                           timesCalled++;
                                       }),
                                       testing::ReturnPointee(&positionUpdate)));
    EXPECT_CALL(m_updateInputQueue, pop).Times(InterlocDumpDTO::MAX_UPDATES_SIZE);

    InterlocUpdate* arrayArg;
    uint16_t arrayLengthArg;
    EXPECT_CALL(m_messageHandlerMock, sendInterlocDump(testing::_, testing::_))
        .Times(1)
        .WillOnce(
            testing::DoAll(testing::SaveArg<0>(&arrayArg), testing::SaveArg<1>(&arrayLengthArg)));

    for (int i = 0; i < InterlocDumpDTO::MAX_UPDATES_SIZE; i++) {
        m_interloc->process();
    }

    EXPECT_EQ(arrayLengthArg, InterlocDumpDTO::MAX_UPDATES_SIZE);
    for (int i = 0; i < InterlocDumpDTO::MAX_UPDATES_SIZE; i++) {
        EXPECT_EQ(arrayArg[i].m_distance, i);
    }
}