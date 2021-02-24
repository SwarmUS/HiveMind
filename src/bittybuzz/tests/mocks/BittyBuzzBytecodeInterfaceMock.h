#ifndef __BITTYBUZZBYTECODEINTERFACEMOCK_H_
#define __BITTYBUZZBYTECODEINTERFACEMOCK_H_

#include <bittybuzz/IBittyBuzzBytecode.h>
#include <gmock/gmock.h>

class BittyBuzzBytecodeInterfaceMock : public IBittyBuzzBytecode {
  public:
    ~BittyBuzzBytecodeInterfaceMock() = default;

    MOCK_METHOD(bbzvm_bcode_fetch_fun, getBytecodeFetchFunction, (), (const override));

    MOCK_METHOD(uint16_t, getBytecodeLength, (), (const override));
};

#endif // __BITTYBUZZBYTECODEINTERFACEMOCK_H_
