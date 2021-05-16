#ifndef __HIVECONNECTHIVEMINDAPIMESSAGEHANDLERINTERFACEMOCK_H_
#define __HIVECONNECTHIVEMINDAPIMESSAGEHANDLERINTERFACEMOCK_H_

#include <gmock/gmock.h>
#include <message-handler/IHiveConnectHiveMindApiMessageHandler.h>

class HiveConnectHiveMindApiMessageHandlerInterfaceMock
    : public IHiveConnectHiveMindApiMessageHandler {

  public:
    ~HiveConnectHiveMindApiMessageHandlerInterfaceMock() override = default;

    MOCK_METHOD(bool,
                handleMessage,
                (uint16_t sourceId, uint16_t destId, const HiveConnectHiveMindApiDTO& message),
                (override));
};

#endif // __HIVECONNECTHIVEMINDAPIMESSAGEHANDLERINTERFACEMOCK_H_
