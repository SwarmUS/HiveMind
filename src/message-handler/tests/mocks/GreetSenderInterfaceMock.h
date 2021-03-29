#ifndef __GREETSENTERINTERFACEMOCK_H_
#define __GREETSENTERINTERFACEMOCK_H_

#include <gmock/gmock.h>
#include <message-handler/IGreetSender.h>

class GreetSenderInterfaceMock : public IGreetSender {
  public:
    ~GreetSenderInterfaceMock() override = default;

    MOCK_METHOD(bool, sendGreet, (), (override));
};

#endif // __GREETSENTERINTERFACEMOCK_H_
