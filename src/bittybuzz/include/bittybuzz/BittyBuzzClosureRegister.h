#ifndef __BITTYBUZZCLOSUREREGISTER_H_
#define __BITTYBUZZCLOSUREREGISTER_H_

#include "IBittyBuzzClosureRegister.h"
#include <bbzvm.h>
#include <cstdint>
#include <functional>
#include <tuple>

class BittyBuzzClosureRegister : public IBittyBuzzClosureRegister {
  public:
    BittyBuzzClosureRegister();
    ~BittyBuzzClosureRegister() = default;

    bool registerFunction(const char* functionName, bbzheap_idx_t functionHeapIdx) override;

    std::optional<bbzheap_idx_t> getFunctionHeapIdx(const char* functionName) const override;

    constexpr static uint16_t m_maxSize = 8;

  private:
    std::array<std::tuple<size_t, bbzheap_idx_t>, m_maxSize> m_closureRegisters;
    uint16_t m_closureRegistersLength = 0;
};

#endif // __BITTYBUZZCLOSUREREGISTER_H_
