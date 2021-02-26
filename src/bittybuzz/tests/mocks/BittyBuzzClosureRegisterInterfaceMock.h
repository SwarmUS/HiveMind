#ifndef __BITTYBUZZCLOSUREREGISTERINTERFACEMOCK_H_
#define __BITTYBUZZCLOSUREREGISTERINTERFACEMOCK_H_

#include <bittybuzz/IBittyBuzzClosureRegister.h>
#include <gmock/gmock.h>

class BittyBuzzClosureRegisterInterfaceMock : public IBittyBuzzClosureRegister {
  public:
    ~BittyBuzzClosureRegisterInterfaceMock() override = default;

    MOCK_METHOD(bool,
                registerFunction,
                (const char* functionName, bbzheap_idx_t functionId),
                (override));

    MOCK_METHOD(std::optional<bbzheap_idx_t>,
                getFunctionHeapIdx,
                (const char* functionName),
                (const override));
};

#endif // __BITTYBUZZCLOSUREREGISTERINTERFACEMOCK_H_
