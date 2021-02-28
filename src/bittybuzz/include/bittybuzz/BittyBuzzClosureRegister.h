#ifndef __BITTYBUZZCLOSUREREGISTER_H_
#define __BITTYBUZZCLOSUREREGISTER_H_

#include "IBittyBuzzClosureRegister.h"
#include "bittybuzz/BittyBuzzSettings.h"
#include <bbzvm.h>
#include <cstdint>
#include <functional>
#include <tuple>

class BittyBuzzClosureRegister : public IBittyBuzzClosureRegister {
  public:
    BittyBuzzClosureRegister();
    ~BittyBuzzClosureRegister() = default;

    bool registerClosure(const char* functionName, bbzheap_idx_t closureHeapIdx) override;

    std::optional<bbzheap_idx_t> getClosureHeapIdx(const char* functionName) const override;

    constexpr static uint16_t m_maxSize = BBZ_CLOSURE_REGISTER_LENGTH;

  private:
    std::array<std::tuple<size_t, bbzheap_idx_t>, m_maxSize> m_closureRegisters;
    uint16_t m_closureRegistersLength = 0;
};

#endif // __BITTYBUZZCLOSUREREGISTER_H_
