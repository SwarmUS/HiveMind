#ifndef __BITTYBUZZVM_H_
#define __BITTYBUZZVM_H_

#include "bittybuzz/IBittyBuzzBytecode.h"
#include "bittybuzz/IBittyBuzzVm.h"
#include "bsp/IBSP.h"
#include "logger/ILogger.h"

class BittyBuzzVm : public IBittyBuzzVm {
  public:
    BittyBuzzVm(const IBittyBuzzBytecode& bytecode, const IBSP& bsp, const ILogger& logger);

    ~BittyBuzzVm() override = default;

    bool step() override;

    bbzvm_state getSate() const override;

    bbzvm_error getError() const override;

  private:
    const IBittyBuzzBytecode& m_bytecode;
    const IBSP& m_bsp;
    const ILogger& m_logger;
};

#endif // __BITTYBUZZVM_H_
