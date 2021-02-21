#ifndef __BITTYBUZZFUNCTIONREGISTER_H_
#define __BITTYBUZZFUNCTIONREGISTER_H_

#include <cstdint>
#include <functional>
#include <optional>
#include <tuple>

class BittyBuzzFunctionRegister {
  public:
    BittyBuzzFunctionRegister();
    ~BittyBuzzFunctionRegister() = default;
    bool registerFunction(const char* functionName, uint16_t functionId);
    std::optional<uint16_t> getFunctionId(const char* functionName);

  private:
    constexpr static uint16_t m_maxSize = 8;
    std::array<std::tuple<size_t, uint16_t>, m_maxSize> m_functionRegisters;
    uint16_t m_functionRegistersLength;
};

#endif // __BITTYBUZZFUNCTIONREGISTER_H_
