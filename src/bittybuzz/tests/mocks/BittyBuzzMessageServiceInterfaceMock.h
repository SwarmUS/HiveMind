#ifndef __BITTYBUZZMESSAGESERVICEINTERFACEMOCK_H_
#define __BITTYBUZZMESSAGESERVICEINTERFACEMOCK_H_

#include <bittybuzz/IBittyBuzzMessageService.h>
#include <gmock/gmock.h>

class BittyBuyzzMessageServiceInterfaceMock : public IBittyBuzzMessageService {
  public:
    ~BittyBuyzzMessageServiceInterfaceMock() override = default;

    MOCK_METHOD(bool,
                callFunction,
                (uint16_t id,
                 const char* functionName,
                 const FunctionCallArgumentDTO* args,
                 uint16_t argsLength),
                (override));
};

#endif // __BITTYBUZZMESSAGESERVICEINTERFACEMOCK_H_
