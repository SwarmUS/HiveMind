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
                 bbzheap_idx_t selfHeapIdx,
                 const BittyBuzzFunctionDescription& description),
                (override));

    MOCK_METHOD(std::optional<std::reference_wrapper<const BittyBuzzRegisteredClosure>>,
                getRegisteredClosure,
                (const char* functionName),
                (const override));

    MOCK_METHOD(std::optional<std::reference_wrapper<const BittyBuzzRegisteredClosure>>,
                getRegisteredClosure,
                (uint16_t idx),
                (const override));
};

#endif // __BITTYBUZZCLOSUREREGISTERINTERFACEMOCK_H_
