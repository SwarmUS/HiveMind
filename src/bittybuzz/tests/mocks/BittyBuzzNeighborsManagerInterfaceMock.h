#ifndef __BITTYBUZZNEIGHBOTSMANAGERINTERFACEMOCK_H_
#define __BITTYBUZZNEIGHBOTSMANAGERINTERFACEMOCK_H_

#include <bittybuzz/IBittyBuzzNeighborsManager.h>
#include <gmock/gmock.h>

class BittyBuzzNeighborsManagerInterfaceMock : public IBittyBuzzNeighborsManager {
  public:
    ~BittyBuzzNeighborsManagerInterfaceMock() override = default;

    MOCK_METHOD(void, updateNeighbors, (), (override));
};

#endif // __BITTYBUZZNEIGHBOTSMANAGERINTERFACEMOCK_H_
