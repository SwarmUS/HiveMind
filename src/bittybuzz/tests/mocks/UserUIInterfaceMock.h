#ifndef USERUIINTERFACEMOCK_H_
#define USERUIINTERFACEMOCK_H_

#include <application-interface/IUserUI.h>

#include <gmock/gmock.h>

class UserUIInterfaceMock : public IUserUI {
  public:
    ~UserUIInterfaceMock() = default;

    MOCK_METHOD(void, setLed, (bool state), (override));

    MOCK_METHOD(void, setSegment, (UserSegment segment), (override));
};

#endif // USERUIINTERFACEMOCK_H_
