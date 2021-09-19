#ifndef __BITTYBUZZCLOSUREREGISTER_H_
#define __BITTYBUZZCLOSUREREGISTER_H_

#include "IBittyBuzzClosureRegister.h"
#include "bittybuzz/BittyBuzzSettings.h"
#include <bbzvm.h>
#include <cpp-common/HashMapStack.h>
#include <cstdint>
#include <functional>
#include <string_view>
#include <tuple>

class BittyBuzzClosureRegister : public IBittyBuzzClosureRegister {
  public:
    ~BittyBuzzClosureRegister() = default;

    bool registerClosure(const char* functionName,
                         bbzheap_idx_t closureHeapIdx,
                         bbzheap_idx_t selfHeapIdx,
                         const BittyBuzzFunctionDescription& description) override;

    void clearClosures() override;

    std::optional<std::reference_wrapper<const BittyBuzzRegisteredClosure>> getRegisteredClosure(
        const char* functionName) const override;

    std::optional<std::reference_wrapper<const BittyBuzzRegisteredClosure>> getRegisteredClosure(
        uint16_t idx) const override;

    uint16_t getRegisteredClosureLength() const override;

    constexpr static uint16_t m_maxSize = BBZ_CLOSURE_REGISTER_LENGTH;

  private:
    HashMapStack<uint16_t, const char*, m_maxSize> m_closureNameRegisters;
    HashMapStack<std::string_view, BittyBuzzRegisteredClosure, m_maxSize> m_closureRegisterMap;
};

#endif // __BITTYBUZZCLOSUREREGISTER_H_
