#include "BittyBuzzClosureRegister.h"
#include <bbzvm.h>
#include <cstring>
#include <string_view>

BittyBuzzRegisteredClosure::BittyBuzzRegisteredClosure(uint16_t closureId,
                                                       BittyBuzzFunctionDescription description,
                                                       bbzheap_idx_t closureHeapIdx,
                                                       bbzheap_idx_t selfHeapIdx) :

    m_closureId(closureId),
    m_closureHeapIdx(closureHeapIdx),
    m_selfHeapIdx(selfHeapIdx),
    m_description(std::move(description)) {}

bool BittyBuzzClosureRegister::registerClosure(const char* functionName,
                                               bbzheap_idx_t closureHeapIdx,
                                               bbzheap_idx_t selfHeapIdx,
                                               const BittyBuzzFunctionDescription& description) {
    bbzobj_t* closure = bbzheap_obj_at(closureHeapIdx);

    if (!bbztype_isclosure(*closure)) {
        return false;
    }

    bbzobj_t* self = bbzheap_obj_at(selfHeapIdx);
    if (!bbztype_istable(*self) && !bbztype_isnil(*self)) {
        return false;
    }

    // Making object permanent
    bbzheap_obj_make_permanent(*closure);
    bbzheap_obj_make_permanent(*self);

    BittyBuzzRegisteredClosure registeredClosure(m_closureRegisterMap.getUsedSpace(), description,
                                                 closureHeapIdx, selfHeapIdx);
    // Handler overwrite
    auto closureOpt = m_closureRegisterMap.at(functionName);

    if (closureOpt) {
        registeredClosure.m_closureId = closureOpt.value().get().m_closureId;
    }

    bool ret = m_closureRegisterMap.upsert(functionName, registeredClosure);
    ret = ret && m_closureNameRegisters.upsert(registeredClosure.m_closureId, functionName);
    return ret;
}

std::optional<std::reference_wrapper<const BittyBuzzRegisteredClosure>> BittyBuzzClosureRegister::
    getRegisteredClosure(const char* functionName) const {
    return m_closureRegisterMap.at(functionName);
}

std::optional<std::reference_wrapper<const BittyBuzzRegisteredClosure>> BittyBuzzClosureRegister::
    getRegisteredClosure(uint16_t idx) const {
    auto functionNameOpt = m_closureNameRegisters.at(idx);
    if (functionNameOpt) {
        return m_closureRegisterMap.at(functionNameOpt.value().get());
    }
    return {};
}

uint16_t BittyBuzzClosureRegister::getRegisteredClosureLength() const {
    return m_closureRegisterMap.getUsedSpace();
}
