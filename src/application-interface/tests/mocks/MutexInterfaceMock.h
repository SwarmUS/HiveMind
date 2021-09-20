#ifndef MUTEXINTERFACEMOCK_H_
#define MUTEXINTERFACEMOCK_H_

#include <IMutex.h>
#include <gmock/gmock.h>

class MutexInterfaceMock : public IMutex {
  public:
    ~MutexInterfaceMock() override = default;

    MOCK_METHOD(bool, lock, (), (override));

    MOCK_METHOD(bool, unlock, (), (override));
};

#endif // MUTEXINTERFACEMOCK_H_
