#include "BittyBuzzClosureRegister.h"
#include <bbzvm.h>
#include <cstring>

BittyBuzzClosureRegister::BittyBuzzClosureRegister() = default;

bool BittyBuzzClosureRegister::registerClosure(const char* functionName,
                                               bbzheap_idx_t closureHeapIdx,
                                               bbzheap_idx_t selfHeapIdx,
                                               const BittyBuzzFunctionDescription& description) {
    if (m_closureRegistersLength >= m_maxSize) {
        return false;
    }

    bbzobj_t* closure = bbzheap_obj_at(closureHeapIdx);

    if (!bbztype_isclosure(*closure)) {
        return false;
    }

    bbzobj_t* self = bbzheap_obj_at(selfHeapIdx);
    if (!bbztype_istable(*self) && !bbztype_isnil(*self)) {
        return false;
    }

    std::string_view functionNameView(functionName);
    size_t functionNameHash = std::hash<std::string_view>{}(functionNameView);

    // Making object permanent
    bbzheap_obj_make_permanent(*closure);
    bbzheap_obj_make_permanent(*self);

    m_closureRegisters[m_closureRegistersLength] = {functionNameHash,
                                                    {closureHeapIdx, selfHeapIdx, description}};
    m_closureRegistersLength++;
    return true;
}

std::optional<std::reference_wrapper<const BittyBuzzRegisteredClosure>> BittyBuzzClosureRegister::
    getRegisteredClosure(const char* functionName) const {

    std::string_view functionNameView(functionName);
    size_t functionNameHash = std::hash<std::string_view>{}(functionNameView);

    for (uint16_t i = 0; i < m_closureRegistersLength; i++) {
        auto [hash, registeredClosure] = m_closureRegisters[i];
        if (functionNameHash == hash) {
            return registeredClosure;
        }
    }

    return {};
}
