#ifndef __BITTYBUZZVM_H_
#define __BITTYBUZZVM_H_

#include "bittybuzz/IBittyBuzzVm.h"

class BittyBuzzVm : public IBittyBuzzVm {
  public:
    BittyBuzzVm() = default;
    ~BittyBuzzVm() override = default;
    void init() override;
    bool step() override;
    bbzvm_state getSate() const override;
    bbzvm_error getError() const override;
};

#endif // __BITTYBUZZVM_H_
