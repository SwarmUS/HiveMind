#ifndef __INTERLOCMESSAGEHANDLERMOCK_H__
#define __INTERLOCMESSAGEHANDLERMOCK_H__

#include "interloc/IInterlocMessageHandler.h"
#include <gmock/gmock.h>

class InterlocMessageHandlerMock : public IInterlocMessageHandler {
  public:
    InterlocMessageHandlerMock() {
        ON_CALL(*this, getDumpEnabled).WillByDefault(testing::Return(false));
    }

    MOCK_METHOD(bool, processMessage, (), (override));
    MOCK_METHOD(bool, getDumpEnabled, (), (const override));
    MOCK_METHOD(bool,
                sendInterlocDump,
                (InterlocUpdate * updatesHistory, uint8_t updatesLength),
                (override));
};

#endif //__INTERLOCMESSAGEHANDLERMOCK_H__
