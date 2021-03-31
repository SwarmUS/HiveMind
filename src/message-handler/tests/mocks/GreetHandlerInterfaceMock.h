#ifndef __GREETHANDLERINTERFACEMOCK_H_
#define __GREETHANDLERINTERFACEMOCK_H_

#include <gmock/gmock.h>
#include <message-handler/IGreetHandler.h>

class GreetHandlerInterfaceMock : public IGreetHandler {
  public:
    ~GreetHandlerInterfaceMock() override = default;

    MOCK_METHOD(bool, greet, (), (override));
};

#endif // __GREETHANDLERINTERFACEMOCK_H_
