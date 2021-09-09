#include "mocks/MutexInterfaceMock.h"
#include "mocks/UserInterfaceMock.h"
#include <application-interface/ApplicationInterface.h>
#include <gtest/gtest.h>

class ApplicationInterfaceFixture : public testing::Test {
  protected:
    ApplicationInterface* m_appInterface;

    testing::StrictMock<MutexInterfaceMock> m_mutexMock;
    UserInterfaceMock m_uiMock;

    void SetUp() override { m_appInterface = new ApplicationInterface(m_uiMock, m_mutexMock); }

    void TearDown() override { delete m_appInterface; }
};

TEST_F(ApplicationInterfaceFixture, ApplicationInterface_setSystemESPHandshaked_default) {
    // Given
    EXPECT_CALL(m_mutexMock, lock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_mutexMock, unlock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_uiMock, setLed(ApplicationInterface::s_espLed, true)).Times(1);

    // Then
    m_appInterface->setSystemESPHandshaked(true);

    // Expect
    EXPECT_TRUE(m_appInterface->getApplicationState().m_systemStates.m_espHandshaked);
}
