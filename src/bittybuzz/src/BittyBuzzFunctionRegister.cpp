#include "BittyBuzzFunctionRegister.h"
#include <cstring>

BittyBuzzFunctionRegister::BittyBuzzFunctionRegister() {}

bool BittyBuzzFunctionRegister::registerFunction(const char* functionName, uint16_t functionId) {
    if (m_functionRegistersLength >= m_maxSize - 1) {
        return false;
    }
    std::string_view functionNameView(functionName);
    size_t functionNameHash = std::hash<std::string_view>{}(functionNameView);

    m_functionRegisters[m_functionRegistersLength] = std::make_tuple(functionNameHash, functionId);
    m_functionRegistersLength++;
    return true;
}

std::optional<uint16_t> BittyBuzzFunctionRegister::getFunctionId(const char* functionName) {
    std::string_view functionNameView(functionName);
    size_t functionNameHash = std::hash<std::string_view>{}(functionNameView);

    for (uint16_t i = 0; i < m_functionRegistersLength; i++) {
        auto [hash, functionId] = m_functionRegisters[i];
        if (functionNameHash == hash) {
            return functionId;
        }
    }

    return {};
}
