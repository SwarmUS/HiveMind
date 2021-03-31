#ifndef __INTERLOCMESSAGEHANDLERMOCK_H__
#define __INTERLOCMESSAGEHANDLERMOCK_H__

#include <gmock/gmock.h>
#include <interloc/IInterlocMessageHandler.h>

class InterlocMessageHandlerMock final : public IInterlocMessageHandler {
  public:
    ~InterlocMessageHandlerMock() = default;

    bool processMessage() override { return true; };

    MOCK_METHOD(bool, notifyCalibrationEnded, (uint16_t initiatorId), (override));
};

#endif //__INTERLOCMESSAGEHANDLERMOCK_H__
