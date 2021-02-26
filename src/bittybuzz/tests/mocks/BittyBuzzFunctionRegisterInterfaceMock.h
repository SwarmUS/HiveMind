#ifndef __BITTYBUZZFUNCTIONREGISTERINTERFACEMOCK_H_
#define __BITTYBUZZFUNCTIONREGISTERINTERFACEMOCK_H_

#include <bittybuzz/IBittyBuzzFunctionRegister.h>
#include <gmock/gmock.h>

class BittyBuzzFunctionRegisterInterfaceMock : public IBittyBuzzFunctionRegister {
  public:
    ~BittyBuzzFunctionRegisterInterfaceMock() override = default;

    MOCK_METHOD(bool,
                registerFunction,
                (const char* functionName, bbzheap_idx_t functionId),
                (override));

    MOCK_METHOD(std::optional<bbzheap_idx_t>,
                getFunctionHeapIdx,
                (const char* functionName),
                (const override));
};

#endif // __BITTYBUZZFUNCTIONREGISTERINTERFACEMOCK_H_
