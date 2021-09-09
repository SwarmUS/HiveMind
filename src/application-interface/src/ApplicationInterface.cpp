#include "ApplicationInterface.h"

ApplicationInterface::ApplicationInterface(IUserInterface& userInterface) :
    m_userInterface(userInterface) {}

void ApplicationInterface::setSystemESPHandshaked(bool handshaked) {
    m_userInterface.setLed(s_espLed, handshaked);
    m_systemStates.m_espHandshaked = true;
}

void ApplicationInterface::setSystemHostHandshaked(bool handshaked) {
    m_userInterface.setLed(s_hostLed, handshaked);
    m_systemStates.m_hostHandshaked = true;
}

void ApplicationInterface::setSystemConnectionState(ConnectionState state) {
    m_systemStates.m_connection = state;
    m_userInterface.
}

void ApplicationInterface::setSystemDeviceState(DeviceState state) {}

void ApplicationInterface::setUserLed(bool state) {}

void ApplicationInterface::setSevenSegment(SevenSegment segment) {}

const SystemStates& ApplicationInterface::getSystemStates() const {}

const UserStates& ApplicationInterface::getUserStates() const {}
