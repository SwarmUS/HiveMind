#ifndef __HIVEMINDAPIREQUESTHANDLERINTERFACEMOCK_H_
#define __HIVEMINDAPIREQUESTHANDLERINTERFACEMOCK_H_

#include <gmock/gmock.h>
#include <message-handler/IHiveMindHostApiRequestHandler.h>

class HiveMindHostApiRequestHandlerInterfaceMock : public IHiveMindHostApiRequestHandler {
  public:
    ~HiveMindHostApiRequestHandlerInterfaceMock() override = default;

    MOCK_METHOD(HiveMindHostApiResponseDTO,
                handleRequest,
                (const HiveMindHostApiRequestDTO& request),
                (override));
};

#endif // __HIVEMINDAPIREQUESTHANDLERINTERFACEMOCK_H_
