
#ifndef __BITTYBUZZBYTECODE_H_
#define __BITTYBUZZBYTECODE_H_

#include "bittybuzz/IBittyBuzzBytecode.h"
#include <bbzvm.h>
#include <cstdint>
#include <logger/ILogger.h>

extern "C" {
#include <main_bytecode.h>
}

class BittyBuzzBytecode : public IBittyBuzzBytecode {
  public:
    BittyBuzzBytecode(const ILogger& logger,
                      const uint8_t* bytecode = bcode,
                      uint16_t bytecodeLength = bcode_size);

    ~BittyBuzzBytecode() = default;

    bbzvm_bcode_fetch_fun getBytecodeFetchFunction() const override;

    uint16_t getBytecodeLength() const override;

  private:
    const uint8_t* m_bytecode;
    uint16_t m_bytecodeSize;
    const ILogger& m_logger;
};

#endif // __BITTYBUZZBYTECODE_H_
