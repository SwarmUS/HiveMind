#include "BittyBuzzClosureRegister.h"
#include <bbzvm.h>
#include <cstring>

BittyBuzzClosureRegister::BittyBuzzClosureRegister() = default;

bool BittyBuzzClosureRegister::registerClosure(const char* functionName,
                                               bbzheap_idx_t closureHeapIdx,
                                               const BittyBuzzFunctionDescription& description) {
    if (m_closureRegistersLength >= m_maxSize) {
        return false;
    }

    bbzobj_t* closure = bbzheap_obj_at(closureHeapIdx);

    if (!bbztype_isclosure(*closure)) {
        return false;
    }

    std::string_view functionNameView(functionName);
    size_t functionNameHash = std::hash<std::string_view>{}(functionNameView);

    // Making object permanent
    bbzheap_obj_make_permanent(*closure);
    m_closureRegisters[m_closureRegistersLength] =
        std::make_tuple(functionNameHash, closureHeapIdx, description);
    m_closureRegistersLength++;
    return true;
}

std::optional<bbzheap_idx_t> BittyBuzzClosureRegister::getClosureHeapIdx(
    const char* functionName) const {
    std::string_view functionNameView(functionName);
    size_t functionNameHash = std::hash<std::string_view>{}(functionNameView);

    for (uint16_t i = 0; i < m_closureRegistersLength; i++) {
        auto [hash, closureHeapIdx, _description] = m_closureRegisters[i];
        if (functionNameHash == hash) {
            return closureHeapIdx;
        }
    }

    return {};
}
