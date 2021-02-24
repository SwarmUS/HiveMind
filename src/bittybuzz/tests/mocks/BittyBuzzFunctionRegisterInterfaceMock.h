#ifndef __BITTYBUZZFUNCTIONREGISTERINTERFACEMOCK_H_
#define __BITTYBUZZFUNCTIONREGISTERINTERFACEMOCK_H_

#include <bittybuzz/IBittyBuzzFunctionRegister.h>
#include <gmock/gmock.h>

class BittyBuzzFunctionRegisterInterfaceMock : public IBittyBuzzFunctionRegister {
  public:
    ~BittyBuzzFunctionRegisterInterfaceMock() override = default;

    MOCK_METHOD(bool,
                registerFunction,
                (const char* functionName, uint16_t functionId),
                (override));

    MOCK_METHOD(std::optional<uint16_t>,
                getFunctionId,
                (const char* functionName),
                (const override));
};

#endif // __BITTYBUZZFUNCTIONREGISTERINTERFACEMOCK_H_
