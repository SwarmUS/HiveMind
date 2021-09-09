#include "ApplicationInterface.h"
#include <LockGuard.h>

ApplicationInterface::ApplicationInterface(IUserInterface& userInterface, IMutex& mutex) :
    m_userInterface(userInterface), m_mutex(mutex) {}

void ApplicationInterface::setSystemESPHandshaked(bool handshaked) {
    LockGuard lock(m_mutex);
    m_userInterface.setLed(s_espLed, handshaked);
    m_states.m_systemStates.m_espHandshaked = true;
}

void ApplicationInterface::setSystemHostHandshaked(bool handshaked) {
    LockGuard lock(m_mutex);
    m_userInterface.setLed(s_hostLed, handshaked);
    m_states.m_systemStates.m_hostHandshaked = true;
}

void ApplicationInterface::setSystemConnectionState(ConnectionState state) {
    LockGuard lock(m_mutex);
    m_states.m_systemStates.m_connection = state;
    switch (state) {
    case ConnectionState::Booting:
        m_userInterface.setRGBLed(RgbColor::YELLOW);
        break;
    case ConnectionState::EthernetHost:
        m_userInterface.setRGBLed(RgbColor::GREEN);
        break;
    case ConnectionState::USBHost:
        m_userInterface.setRGBLed(RgbColor::BLUE);
        break;
    case ConnectionState::Error:
        m_userInterface.setRGBLed(RgbColor::RED);
        break;
    case ConnectionState::Unconnected:
        m_userInterface.setRGBLed(RgbColor::ORANGE);
        break;
    }
}

void ApplicationInterface::setSystemDeviceState(DeviceState state) {
    LockGuard lock(m_mutex);
    m_states.m_systemStates.m_device = state;

    uint8_t userHex = static_cast<uint8_t>(m_states.m_userStates.m_userSegment);
    uint8_t maskedUserHex = 0xff & userHex; // so the number start with 0x0X

    uint8_t value = static_cast<uint8_t>(state) << 4;

    value = value & maskedUserHex;
    m_userInterface.setHexDisplay(value);
}

void ApplicationInterface::setUserLed(bool state) {
    LockGuard lock(m_mutex);
    m_states.m_userStates.m_userLed = state;
    m_userInterface.setLed(s_userLed, state);
}

void ApplicationInterface::setUserSegment(UserSegment segment) {
    LockGuard lock(m_mutex);
    m_states.m_userStates.m_userSegment = segment;
    setSystemDeviceState(m_states.m_systemStates.m_device);
}

SystemStates ApplicationInterface::getSystemStates() const {
    LockGuard lock(m_mutex);
    return m_states.m_systemStates;
}

UserStates ApplicationInterface::getUserStates() const {
    LockGuard lock(m_mutex);
    return m_states.m_userStates;
}

ApplicationStates ApplicationInterface::getApplicationState() const {
    LockGuard lock(m_mutex);
    return m_states;
}
