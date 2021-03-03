#ifndef __MESSAGEHANDLERINTERFACEMOCK_H_
#define __MESSAGEHANDLERINTERFACEMOCK_H_

#include <gmock/gmock.h>
#include <message-handler/IMessageHandler.h>

class MessageHandlerInterfaceMock : public IMessageHandler {
  public:
    ~MessageHandlerInterfaceMock() override = default;

    MOCK_METHOD(bool, handleMessage, (const MessageDTO& message), (override));
};

#endif // __MESSAGEHANDLERINTERFACEMOCK_H_
