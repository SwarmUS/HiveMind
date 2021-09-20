#include "mocks/ApplicationInterfaceMock.h"
#include <application-interface/UserUI.h>
#include <gtest/gtest.h>

class UserUIFixture : public testing::Test {
  protected:
    UserUI* m_userUI;

    testing::StrictMock<ApplicationInterfaceMock> m_appInterfaceMock;

    void SetUp() override { m_userUI = new UserUI(m_appInterfaceMock); }
    void TearDown() override { delete m_userUI; }
};

TEST_F(UserUIFixture, UserUI_setLed) {
    // Given
    EXPECT_CALL(m_appInterfaceMock, setUserLed(true)).Times(1);

    // Then
    m_userUI->setLed(true);

    // Expect
}

TEST_F(UserUIFixture, UserUI_setSegment) {
    // Given
    EXPECT_CALL(m_appInterfaceMock, setUserSegment(UserSegment::A)).Times(1);

    // Then
    m_userUI->setSegment(UserSegment::A);

    // Expect
}
