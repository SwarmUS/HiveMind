#ifndef __HIVEMINDHOSTDESERIALIZERINTERFACEMOCK_H_
#define __HIVEMINDHOSTDESERIALIZERINTERFACEMOCK_H_

#include <gmock/gmock.h>
#include <hivemind-host/IHiveMindHostDeserializer.h>

class HiveMindHostDeserializerInterfaceMock final : public IHiveMindHostDeserializer {
  public:
    ~HiveMindHostDeserializerInterfaceMock() override = default;

    MOCK_METHOD(bool, deserializeFromStream, (MessageDTO & message), (override));
};

#endif // __HIVEMINDHOSTDESERIALIZERINTERFACEMOCK_H_
