#ifndef __BITTYBUZZCLOSUREREGISTERINTERFACEMOCK_H_
#define __BITTYBUZZCLOSUREREGISTERINTERFACEMOCK_H_

#include <bittybuzz/IBittyBuzzClosureRegister.h>
#include <gmock/gmock.h>

class BittyBuzzClosureRegisterInterfaceMock : public IBittyBuzzClosureRegister {
  public:
    ~BittyBuzzClosureRegisterInterfaceMock() override = default;

    MOCK_METHOD(bool,
                registerClosure,
                (const char* functionName,
                 bbzheap_idx_t closureHeapIdx,
                 const BittyBuzzFunctionDescription& description),
                (override));

    MOCK_METHOD(std::optional<bbzheap_idx_t>,
                getClosureHeapIdx,
                (const char* functionName),
                (const override));
};

#endif // __BITTYBUZZCLOSUREREGISTERINTERFACEMOCK_H_
