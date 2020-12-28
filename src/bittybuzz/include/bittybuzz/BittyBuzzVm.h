#ifndef __BITTYBUZZVM_H_
#define __BITTYBUZZVM_H_

#include "bittybuzz/IBittyBuzzBytecode.h"
#include "bittybuzz/IBittyBuzzVm.h"

class BittyBuzzVm : public IBittyBuzzVm {
  public:
    BittyBuzzVm(const IBittyBuzzBytecode&);

    ~BittyBuzzVm() override = default;

    bool step() override;

    bbzvm_state getSate() const override;

    bbzvm_error getError() const override;

  private:
    const IBittyBuzzBytecode& m_bytecode;
};

#endif // __BITTYBUZZVM_H_
