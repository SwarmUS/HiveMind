#ifndef __HIVEMINDAPIREQUESTHANDLERINTERFACEMOCK_H_
#define __HIVEMINDAPIREQUESTHANDLERINTERFACEMOCK_H_

#include <gmock/gmock.h>
#include <message-handler/IHiveMindApiRequestHandler.h>

class HiveMindApiRequestHandlerInterfaceMock : public IHiveMindApiRequestHandler {
  public:
    ~HiveMindApiRequestHandlerInterfaceMock() override = default;

    MOCK_METHOD(HiveMindApiResponseDTO,
                handleRequest,
                (const HiveMindApiRequestDTO& request),
                (override));
};

#endif // __HIVEMINDAPIREQUESTHANDLERINTERFACEMOCK_H_