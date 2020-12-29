
#ifndef __BITTYBUZZBYTECODE_H_
#define __BITTYBUZZBYTECODE_H_

#include "bittybuzz/IBittyBuzzBytecode.h"
#include <bbzvm.h>
#include <cstdint>
#include <logger/ILogger.h>


class BittyBuzzBytecode : public IBittyBuzzBytecode {
  public:
    BittyBuzzBytecode(const ILogger& logger,
                      const uint8_t* bytecode,
                      uint16_t bytecodeLength);

    ~BittyBuzzBytecode() = default;

    bbzvm_bcode_fetch_fun getBytecodeFetchFunction() const override;

    uint16_t getBytecodeLength() const override;
    
  private:
    const uint8_t* m_bytecode;
    const ILogger& m_logger;
    uint16_t m_bytecodeSize;

};

#endif // __BITTYBUZZBYTECODE_H_
