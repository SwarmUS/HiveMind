#include "BittyBuzzFunctionRegister.h"
#include <bbzvm.h>
#include <cstring>

BittyBuzzFunctionRegister::BittyBuzzFunctionRegister() = default;

bool BittyBuzzFunctionRegister::registerFunction(const char* functionName,
                                                 bbzheap_idx_t functionHeapIdx) {
    if (m_functionRegistersLength >= m_maxSize) {
        return false;
    }

    bbzobj_t* closure = bbzheap_obj_at(functionHeapIdx);

    if (!bbztype_isclosure(*closure)) {
        return false;
    }

    std::string_view functionNameView(functionName);
    size_t functionNameHash = std::hash<std::string_view>{}(functionNameView);

    // Making object permanent
    bbzheap_obj_make_permanent(*closure);
    m_functionRegisters[m_functionRegistersLength] =
        std::make_tuple(functionNameHash, functionHeapIdx);
    m_functionRegistersLength++;
    return true;
}

std::optional<bbzheap_idx_t> BittyBuzzFunctionRegister::getFunctionHeapIdx(
    const char* functionName) const {
    std::string_view functionNameView(functionName);
    size_t functionNameHash = std::hash<std::string_view>{}(functionNameView);

    for (uint16_t i = 0; i < m_functionRegistersLength; i++) {
        auto [hash, functionHeapIdx] = m_functionRegisters[i];
        if (functionNameHash == hash) {
            return functionHeapIdx;
        }
    }

    return {};
}
