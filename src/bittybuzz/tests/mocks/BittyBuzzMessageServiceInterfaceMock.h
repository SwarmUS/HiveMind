#ifndef __BITTYBUZZMESSAGESERVICEINTERFACEMOCK_H_
#define __BITTYBUZZMESSAGESERVICEINTERFACEMOCK_H_

#include <bittybuzz/IBittyBuzzMessageService.h>
#include <gmock/gmock.h>

class BittyBuyzzMessageServiceInterfaceMock : public IBittyBuzzMessageService {
  public:
    ~BittyBuyzzMessageServiceInterfaceMock() override = default;

    MOCK_METHOD(bool,
                callHostFunction,
                (uint16_t hostId,
                 const char* functionName,
                 const FunctionCallArgumentDTO* args,
                 uint16_t argsLength),
                (override));

    MOCK_METHOD(bool, sendBuzzMessage, (const BuzzMessageDTO& msg), (override));
};

#endif // __BITTYBUZZMESSAGESERVICEINTERFACEMOCK_H_
