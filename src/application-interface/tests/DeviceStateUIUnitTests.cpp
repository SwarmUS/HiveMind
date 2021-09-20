#include "mocks/ApplicationInterfaceMock.h"
#include <application-interface/DeviceStateUI.h>
#include <gtest/gtest.h>

class DeviceStateUIFixture : public testing::Test {
  protected:
    DeviceStateUI* m_deviceStateUI;

    testing::StrictMock<ApplicationInterfaceMock> m_appInterfaceMock;

    void SetUp() override { m_deviceStateUI = new DeviceStateUI(m_appInterfaceMock); }
    void TearDown() override { delete m_deviceStateUI; }
};

TEST_F(DeviceStateUIFixture, DeviceStateUI_setDeviceState) {
    // Given
    EXPECT_CALL(m_appInterfaceMock, setSystemDeviceState(DeviceState::ErrorVMMem)).Times(1);

    // Then
    m_deviceStateUI->setDeviceState(DeviceState::ErrorVMMem);

    // Expect
}
