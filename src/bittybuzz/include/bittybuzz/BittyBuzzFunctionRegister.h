#ifndef __BITTYBUZZFUNCTIONREGISTER_H_
#define __BITTYBUZZFUNCTIONREGISTER_H_

#include "IBittyBuzzFunctionRegister.h"
#include <cstdint>
#include <functional>
#include <tuple>

class BittyBuzzFunctionRegister : IBittyBuzzFunctionRegister {
  public:
    BittyBuzzFunctionRegister();
    ~BittyBuzzFunctionRegister() = default;

    bool registerFunction(const char* functionName, uint16_t functionId) override;

    std::optional<uint16_t> getFunctionId(const char* functionName) const override;

    constexpr static uint16_t m_maxSize = 8;

  private:
    std::array<std::tuple<size_t, uint16_t>, m_maxSize> m_functionRegisters;
    uint16_t m_functionRegistersLength = 0;
};

#endif // __BITTYBUZZFUNCTIONREGISTER_H_
