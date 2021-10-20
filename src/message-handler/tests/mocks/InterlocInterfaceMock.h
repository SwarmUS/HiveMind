#ifndef __INTERLOCINTERFACEMOCK_H_
#define __INTERLOCINTERFACEMOCK_H_

#include <gmock/gmock.h>
#include <interloc/IInterloc.h>

class InterlocInterfaceMock : public IInterloc {
  public:
    virtual ~InterlocInterfaceMock() = default;

    MOCK_METHOD(std::optional<RelativePosition>,
                getRobotPosition,
                (uint16_t robotId),
                (const override));
    MOCK_METHOD(bool, isLineOfSight, (uint16_t robotId), (const override));

    MOCK_METHOD((const PositionsTable&), getPositionsTable, (), (const override));

    MOCK_METHOD(void, process, (), (override));
};

#endif // __INTERLOCINTERFACEMOCK_H_
