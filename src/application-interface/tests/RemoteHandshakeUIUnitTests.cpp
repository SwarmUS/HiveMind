
#include "mocks/ApplicationInterfaceMock.h"
#include <application-interface/RemoteHandshakeUI.h>
#include <gtest/gtest.h>

class RemoteHandshakeUIFixture : public testing::Test {
  protected:
    RemoteHandshakeUI* m_hostHandshakeUI;

    testing::StrictMock<ApplicationInterfaceMock> m_appInterfaceMock;

    void SetUp() override { m_hostHandshakeUI = new RemoteHandshakeUI(m_appInterfaceMock); }
    void TearDown() override { delete m_hostHandshakeUI; }
};

TEST_F(RemoteHandshakeUIFixture, RemoteHandshakeUI_handshake_true) {
    // Given
    EXPECT_CALL(m_appInterfaceMock, setSystemRemoteHandshaked(true)).Times(1);

    // Then
    m_hostHandshakeUI->handshake(true);

    // Expect
}

TEST_F(RemoteHandshakeUIFixture, RemoteHandshakeUI_handshake_false) {
    // Given
    EXPECT_CALL(m_appInterfaceMock, setSystemRemoteHandshaked(false)).Times(1);

    // Then
    m_hostHandshakeUI->handshake(false);

    // Expect
}
