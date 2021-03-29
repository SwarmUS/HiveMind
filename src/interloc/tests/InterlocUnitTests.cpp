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
    std::optional<RobotPosition> ret = m_interloc->getRobotPosition(robotId);

    // Expect
    EXPECT_FALSE(ret.has_value());
}

TEST_F(InterlocFixture, Interloc_getPosition_pushData_robotInList) {
    // Given
    uint16_t robotId = 42;
    RobotPosition position = {.m_robotId = robotId,
                              .m_distance = 42.24,
                              .m_relativeOrientation = 69.0,
                              .m_isInLineOfSight = true};

    // Then
    m_interlocManagerMock->m_callback(position);
    std::optional<RobotPosition> ret = m_interloc->getRobotPosition(robotId);

    // Expect
    EXPECT_TRUE(ret.has_value());
}

TEST_F(InterlocFixture, Interloc_getPosition_validData) {
    // Given
    uint16_t robotId = 42;
    RobotPosition position = {.m_robotId = robotId,
                              .m_distance = 42.24,
                              .m_relativeOrientation = 69.0,
                              .m_isInLineOfSight = true};

    // Then
    m_interlocManagerMock->m_callback(position);
    std::optional<RobotPosition> ret = m_interloc->getRobotPosition(robotId);

    // Expect
    EXPECT_EQ(ret->m_distance, position.m_distance);
    EXPECT_EQ(ret->m_relativeOrientation, position.m_relativeOrientation);
    EXPECT_EQ(ret->m_isInLineOfSight, position.m_isInLineOfSight);
}

TEST_F(InterlocFixture, Interloc_getPosition_updateData_validData) {
    // Given
    uint16_t robotId = 42;
    RobotPosition position1 = {.m_robotId = robotId,
                               .m_distance = 42.24,
                               .m_relativeOrientation = 69.0,
                               .m_isInLineOfSight = true};
    RobotPosition position2 = {.m_robotId = robotId,
                               .m_distance = 43.24,
                               .m_relativeOrientation = 70.0,
                               .m_isInLineOfSight = true};

    // Then
    m_interlocManagerMock->m_callback(position1);
    m_interlocManagerMock->m_callback(position2);
    std::optional<RobotPosition> ret = m_interloc->getRobotPosition(robotId);

    // Expect
    EXPECT_EQ(ret->m_distance, position2.m_distance);
    EXPECT_EQ(ret->m_relativeOrientation, position2.m_relativeOrientation);
    EXPECT_EQ(ret->m_isInLineOfSight, position2.m_isInLineOfSight);
}