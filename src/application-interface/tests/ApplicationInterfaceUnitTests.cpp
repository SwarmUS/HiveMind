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

TEST_F(ApplicationInterfaceFixture, ApplicationInterface_setSystemESPHandshaked_true) {
    // Given
    EXPECT_CALL(m_mutexMock, lock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_mutexMock, unlock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_uiMock, setLed(ApplicationInterface::s_remoteLed, true)).Times(1);

    // Then
    m_appInterface->setSystemRemoteHandshaked(true);

    // Expect
    EXPECT_TRUE(m_appInterface->getApplicationState().m_systemStates.m_remoteHandshaked);
}

TEST_F(ApplicationInterfaceFixture, ApplicationInterface_setSystemESPHandshaked_false) {
    // Given
    EXPECT_CALL(m_mutexMock, lock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_mutexMock, unlock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_uiMock, setLed(ApplicationInterface::s_remoteLed, false)).Times(1);

    // Then
    m_appInterface->setSystemRemoteHandshaked(false);

    // Expect
    EXPECT_FALSE(m_appInterface->getApplicationState().m_systemStates.m_remoteHandshaked);
}

TEST_F(ApplicationInterfaceFixture, ApplicationInterface_setSystemHostHandshaked_true) {
    // Given
    EXPECT_CALL(m_mutexMock, lock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_mutexMock, unlock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_uiMock, setLed(ApplicationInterface::s_hostLed, true)).Times(1);

    // Then
    m_appInterface->setSystemHostHandshaked(true);

    // Expect
    EXPECT_TRUE(m_appInterface->getApplicationState().m_systemStates.m_hostHandshaked);
}

TEST_F(ApplicationInterfaceFixture, ApplicationInterface_setSystemHostHandshaked_false) {
    // Given
    EXPECT_CALL(m_mutexMock, lock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_mutexMock, unlock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_uiMock, setLed(ApplicationInterface::s_hostLed, false)).Times(1);

    // Then
    m_appInterface->setSystemHostHandshaked(false);

    // Expect
    EXPECT_FALSE(m_appInterface->getApplicationState().m_systemStates.m_hostHandshaked);
}

TEST_F(ApplicationInterfaceFixture, ApplicationInterface_setSystemConnectionState_Booting) {
    // Given
    EXPECT_CALL(m_mutexMock, lock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_mutexMock, unlock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_uiMock, setRGBLed(RgbColor::YELLOW)).Times(1);

    // Then
    m_appInterface->setSystemConnectionState(ConnectionState::Booting);

    // Expect
    EXPECT_EQ(m_appInterface->getApplicationState().m_systemStates.m_connection,
              ConnectionState::Booting);
}

TEST_F(ApplicationInterfaceFixture, ApplicationInterface_setSystemConnectionState_EthernetHost) {
    // Given
    EXPECT_CALL(m_mutexMock, lock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_mutexMock, unlock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_uiMock, setRGBLed(RgbColor::GREEN)).Times(1);

    // Then
    m_appInterface->setSystemConnectionState(ConnectionState::Ethernet);

    // Expect
    EXPECT_EQ(m_appInterface->getApplicationState().m_systemStates.m_connection,
              ConnectionState::Ethernet);
}

TEST_F(ApplicationInterfaceFixture, ApplicationInterface_setSystemConnectionState_USBHost) {
    // Given
    EXPECT_CALL(m_mutexMock, lock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_mutexMock, unlock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_uiMock, setRGBLed(RgbColor::BLUE)).Times(1);

    // Then
    m_appInterface->setSystemConnectionState(ConnectionState::USB);

    // Expect
    EXPECT_EQ(m_appInterface->getApplicationState().m_systemStates.m_connection,
              ConnectionState::USB);
}

TEST_F(ApplicationInterfaceFixture, ApplicationInterface_setSystemConnectionState_Error) {
    // Given
    EXPECT_CALL(m_mutexMock, lock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_mutexMock, unlock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_uiMock, setRGBLed(RgbColor::RED)).Times(1);

    // Then
    m_appInterface->setSystemConnectionState(ConnectionState::Error);

    // Expect
    EXPECT_EQ(m_appInterface->getApplicationState().m_systemStates.m_connection,
              ConnectionState::Error);
}

TEST_F(ApplicationInterfaceFixture, ApplicationInterface_setSystemConnectionState_Unconnected) {
    // Given
    EXPECT_CALL(m_mutexMock, lock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_mutexMock, unlock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_uiMock, setRGBLed(RgbColor::ORANGE)).Times(1);

    // Then
    m_appInterface->setSystemConnectionState(ConnectionState::Unconnected);

    // Expect
    EXPECT_EQ(m_appInterface->getApplicationState().m_systemStates.m_connection,
              ConnectionState::Unconnected);
}

TEST_F(ApplicationInterfaceFixture, ApplicationInterface_setSystemDeviceState_valid) {
    // Given
    EXPECT_CALL(m_mutexMock, lock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_mutexMock, unlock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_uiMock, setHexDisplay(static_cast<uint>(DeviceState::ErrorVMVstig) << 4))
        .Times(1);

    // Then
    m_appInterface->setSystemDeviceState(DeviceState::ErrorVMVstig);

    // Expect
    EXPECT_EQ(m_appInterface->getApplicationState().m_systemStates.m_device,
              DeviceState::ErrorVMVstig);
}

TEST_F(ApplicationInterfaceFixture, ApplicationInterface_setSystemDeviceState_invalid) {
    // Given
    EXPECT_CALL(m_mutexMock, lock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_mutexMock, unlock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_uiMock, setHexDisplay(0xf0)).Times(1);

    // Then
    DeviceState invalidState = (DeviceState)0xff;
    m_appInterface->setSystemDeviceState(invalidState);

    // Expect
    EXPECT_EQ(m_appInterface->getApplicationState().m_systemStates.m_device, (DeviceState)0xff);
}

TEST_F(ApplicationInterfaceFixture, ApplicationInterface_setUserLed_true) {
    // Given
    EXPECT_CALL(m_mutexMock, lock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_mutexMock, unlock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_uiMock, setLed(ApplicationInterface::s_userLed, true)).Times(1);

    // Then
    m_appInterface->setUserLed(true);

    // Expect
    EXPECT_TRUE(m_appInterface->getApplicationState().m_userStates.m_userLed);
}

TEST_F(ApplicationInterfaceFixture, ApplicationInterface_setUserLed_false) {
    // Given
    EXPECT_CALL(m_mutexMock, lock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_mutexMock, unlock()).Times(2).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_uiMock, setLed(ApplicationInterface::s_userLed, false)).Times(1);

    // Then
    m_appInterface->setUserLed(false);

    // Expect
    EXPECT_FALSE(m_appInterface->getApplicationState().m_userStates.m_userLed);
}

TEST_F(ApplicationInterfaceFixture, ApplicationInterface_setUserSegment_valid) {
    // Given
    EXPECT_CALL(m_mutexMock, lock()).Times(3).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_mutexMock, unlock()).Times(3).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_uiMock, setHexDisplay(0x0C)).Times(1);

    // Then
    m_appInterface->setUserSegment(UserSegment::C);

    // Expect
    EXPECT_EQ(m_appInterface->getApplicationState().m_userStates.m_userSegment, UserSegment::C);
}

TEST_F(ApplicationInterfaceFixture, ApplicationInterface_setUserSegment_invalid) {
    // Given
    EXPECT_CALL(m_mutexMock, lock()).Times(3).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_mutexMock, unlock()).Times(3).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_uiMock, setHexDisplay(0x0C)).Times(1);

    // Then
    m_appInterface->setUserSegment((UserSegment)0XFC);

    // Expect
    EXPECT_EQ(m_appInterface->getApplicationState().m_userStates.m_userSegment, (UserSegment)0xFC);
}
