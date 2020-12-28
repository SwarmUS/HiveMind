
#ifndef __BITTYBUZZBYTECODE_H_
#define __BITTYBUZZBYTECODE_H_

#include "bittybuzz/IBittyBuzzBytecode.h"
#include <bbzvm.h>
#include <cstdint>

class BittyBuzzBytecode : public IBittyBuzzBytecode {
  public:
    BittyBuzzBytecode() = default;

    ~BittyBuzzBytecode() = default;

    bbzvm_bcode_fetch_fun getBytecodeFetchFunction() const override;

    uint16_t getBytecodeLength() const override;
};

#endif // __BITTYBUZZBYTECODE_H_
