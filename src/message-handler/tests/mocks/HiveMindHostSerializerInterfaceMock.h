#ifndef __HIVEMINDHOSTSERIALIZERINTERFACEMOCK_H_
#define __HIVEMINDHOSTSERIALIZERINTERFACEMOCK_H_

#include <gmock/gmock.h>
#include <pheromones/IHiveMindHostSerializer.h>

class HiveMindHostSerializerInterfaceMock : public IHiveMindHostSerializer {
  public:
    ~HiveMindHostSerializerInterfaceMock() override = default;

    MOCK_METHOD(bool, serializeToStream, (const MessageDTO& message), (override));
};

#endif // __HIVEMINDHOSTSERIALIZERINTERFACEMOCK_H_
