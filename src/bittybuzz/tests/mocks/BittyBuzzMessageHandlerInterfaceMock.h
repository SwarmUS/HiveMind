#ifndef __BITTYBUZZMESSAGEHANDLERINTERFACEMOCK_H_
#define __BITTYBUZZMESSAGEHANDLERINTERFACEMOCK_H_

#include <bittybuzz/IBittyBuzzMessageHandler.h>
#include <gmock/gmock.h>

class BittyBuzzMessageHandlerInterfaceMock : public IBittyBuzzMessageHandler {
  public:
    ~BittyBuzzMessageHandlerInterfaceMock() override = default;

    MOCK_METHOD(bool, processMessage, (), (override));

    MOCK_METHOD(void, clearMessages, (), (override));

    MOCK_METHOD(uint16_t, messageQueueLength, (), (const override));
};

#endif // __BITTYBUZZMESSAGEHANDLERINTERFACEMOCK_H_
