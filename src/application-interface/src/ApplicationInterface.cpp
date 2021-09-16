#include "ApplicationInterface.h"
#include <LockGuard.h>

ApplicationInterface::ApplicationInterface(IUserInterface& userInterface, IMutex& mutex) :
    m_userInterface(userInterface), m_mutex(mutex) {}

void ApplicationInterface::setSystemRemoteHandshaked(bool handshaked) {
    LockGuard lock(m_mutex);
    m_userInterface.setLed(s_remoteLed, handshaked);
    m_states.m_systemStates.m_remoteHandshaked = handshaked;
}

void ApplicationInterface::setSystemHostHandshaked(bool handshaked) {
    LockGuard lock(m_mutex);
    m_userInterface.setLed(s_hostLed, handshaked);
    m_states.m_systemStates.m_hostHandshaked = handshaked;
}

void ApplicationInterface::setSystemConnectionState(ConnectionState state) {
    LockGuard lock(m_mutex);
    m_states.m_systemStates.m_connection = state;
    switch (state) {
    case ConnectionState::Booting:
        m_userInterface.setRGBLed(RgbColor::YELLOW);
        break;
    case ConnectionState::Ethernet:
        m_userInterface.setRGBLed(RgbColor::GREEN);
        break;
    case ConnectionState::USB:
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
    uint8_t value = static_cast<uint8_t>(state) << 4;
    value = (0xf0 & value) + (0x0f & userHex); // Bound the values
    m_userInterface.setHexDisplay(value);
}

void ApplicationInterface::setSystemButtonCallback(Button button,
                                                   buttonCallbackFunction_t callback,
                                                   void* context) {
    m_userInterface.setButtonCallback(button, callback, context);
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
