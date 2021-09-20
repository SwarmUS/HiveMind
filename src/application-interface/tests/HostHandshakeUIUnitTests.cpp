#include "mocks/ApplicationInterfaceMock.h"
#include <application-interface/HostHandshakeUI.h>
#include <gtest/gtest.h>

class HostHandshakeUIFixture : public testing::Test {
  protected:
    HostHandshakeUI* m_hostHandshakeUI;

    testing::StrictMock<ApplicationInterfaceMock> m_appInterfaceMock;

    void SetUp() override { m_hostHandshakeUI = new HostHandshakeUI(m_appInterfaceMock); }
    void TearDown() override { delete m_hostHandshakeUI; }
};

TEST_F(HostHandshakeUIFixture, HostHandshakeUI_handshake_true) {
    // Given
    EXPECT_CALL(m_appInterfaceMock, setSystemHostHandshaked(true)).Times(1);

    // Then
    m_hostHandshakeUI->handshake(true);

    // Expect
}

TEST_F(HostHandshakeUIFixture, HostHandshakeUI_handshake_false) {
    // Given
    EXPECT_CALL(m_appInterfaceMock, setSystemHostHandshaked(false)).Times(1);

    // Then
    m_hostHandshakeUI->handshake(false);

    // Expect
}
