#ifndef __BITTYBUZZFUNCTIONREGISTER_H_
#define __BITTYBUZZFUNCTIONREGISTER_H_

#include "IBittyBuzzFunctionRegister.h"
#include <bbzvm.h>
#include <cstdint>
#include <functional>
#include <tuple>

class BittyBuzzFunctionRegister : public IBittyBuzzFunctionRegister {
  public:
    BittyBuzzFunctionRegister();
    ~BittyBuzzFunctionRegister() = default;

    bool registerFunction(const char* functionName, bbzheap_idx_t functionHeapIdx) override;

    std::optional<bbzheap_idx_t> getFunctionHeapIdx(const char* functionName) const override;

    constexpr static uint16_t m_maxSize = 8;

  private:
    std::array<std::tuple<size_t, bbzheap_idx_t>, m_maxSize> m_functionRegisters;
    uint16_t m_functionRegistersLength = 0;
};

#endif // __BITTYBUZZFUNCTIONREGISTER_H_
