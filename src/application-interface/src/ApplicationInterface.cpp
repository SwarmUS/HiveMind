#include "ApplicationInterface.h"


 
    bool ApplicationInterface::setSystemESPHandshaked(bool handshaked) {}

    bool ApplicationInterface::setSystemHostHandshaked(bool handshaked) {}

    bool ApplicationInterface::setSystemConnectionState(ConnectionState state) {}

    bool ApplicationInterface::setSystemDeviceState(DeviceState state) {}

bool ApplicationInterface::setUserLed(bool state) {}

bool ApplicationInterface::setSevenSegment(SevenSegment segment){}

    const SystemStates& ApplicationInterface::getSystemStates() {}

const UserStates& ApplicationInterface::getUserStates() {}

