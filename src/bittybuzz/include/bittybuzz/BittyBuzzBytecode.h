
#ifndef __BITTYBUZZBYTECODE_H_
#define __BITTYBUZZBYTECODE_H_

#include "bittybuzz/IBittyBuzzBytecode.h"
#include <bbzvm.h>
#include <cstdint>
#include <logger/ILogger.h>

class BittyBuzzBytecode : public IBittyBuzzBytecode {
  public:
    BittyBuzzBytecode(ILogger& logger, const uint8_t* bytecode, uint16_t bytecodeLength);

    ~BittyBuzzBytecode() = default;

    bbzvm_bcode_fetch_fun getBytecodeFetchFunction() const override;

    uint16_t getBytecodeLength() const override;

  private:
    ILogger& m_logger;
    const uint8_t* m_bytecode;
    const uint16_t m_bytecodeLength;
};

#endif // __BITTYBUZZBYTECODE_H_
