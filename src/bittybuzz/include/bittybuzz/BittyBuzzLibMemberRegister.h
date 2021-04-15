#ifndef __BITTYBUZZLIBMEMBERREGISTER_H_
#define __BITTYBUZZLIBMEMBERREGISTER_H_

#include <bbzvm.h>
#include <variant>

class BittyBuzzLibMemberRegister {
  public:
    BittyBuzzLibMemberRegister(uint16_t strId, std::variant<bbzvm_funp, float, int16_t> value);
    uint16_t getStringId() const;
    std::variant<bbzvm_funp, float, int16_t> getValue() const;

  private:
    uint16_t m_strId;
    std::variant<bbzvm_funp, float, int16_t> m_value;
};

#endif // __BITTYBUZZLIBMEMBERREGISTER_H_
