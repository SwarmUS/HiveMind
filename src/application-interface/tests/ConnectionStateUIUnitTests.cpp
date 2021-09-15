#include "mocks/ApplicationInterfaceMock.h"
#include <application-interface/ConnectionStateUI.h>
#include <gtest/gtest.h>

class ConnectionStateUIFixture : public testing::Test {
  protected:
    ConnectionStateUI* m_connectionStateUI;

    testing::StrictMock<ApplicationInterfaceMock> m_appInterfaceMock;

    void SetUp() override { m_connectionStateUI = new ConnectionStateUI(m_appInterfaceMock); }
    void TearDown() override { delete m_connectionStateUI; }
};

TEST_F(ConnectionStateUIFixture, ConnectionStateUI_setConnectionState) {
    // Given
    EXPECT_CALL(m_appInterfaceMock, setSystemConnectionState(ConnectionState::Ethernet)).Times(1);

    // Then
    m_connectionStateUI->setConnectionState(ConnectionState::Ethernet);

    // Expect
}
