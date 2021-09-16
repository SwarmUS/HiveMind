#ifndef APPLICATIONINTERFACEMOCK_H_
#define APPLICATIONINTERFACEMOCK_H_

#include <application-interface/IApplicationInterface.h>
#include <gmock/gmock.h>

class ApplicationInterfaceMock : public IApplicationInterface {
  public:
    ~ApplicationInterfaceMock() = default;
    MOCK_METHOD(void, setSystemRemoteHandshaked, (bool handshaked), (override));
    MOCK_METHOD(void, setSystemHostHandshaked, (bool handshaked), (override));
    MOCK_METHOD(void, setSystemConnectionState, (ConnectionState state), (override));
    MOCK_METHOD(void, setSystemDeviceState, (DeviceState state), (override));
    MOCK_METHOD(void,
                setSystemButtonCallback,
                (Button button, buttonCallbackFunction_t callback, void* context),
                (override));
    MOCK_METHOD(void, setUserLed, (bool state), (override));
    MOCK_METHOD(void, setUserSegment, (UserSegment segment), (override));
    MOCK_METHOD(SystemStates, getSystemStates, (), (const override));
    MOCK_METHOD(UserStates, getUserStates, (), (const override));
    MOCK_METHOD(ApplicationStates, getApplicationState, (), (const override));
};

#endif // APPLICATIONINTERFACEMOCK_H_
