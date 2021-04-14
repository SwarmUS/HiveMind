#include "BittyBuzzLibMemberRegister.h"

BittyBuzzLibMemberRegister::BittyBuzzLibMemberRegister(
    uint16_t strId, std::variant<bbzvm_funp, float, int16_t> value) :
    m_strId(strId), m_value(value) {}

uint16_t BittyBuzzLibMemberRegister::getStringId() const { return m_strId; }

std::variant<bbzvm_funp, float, int16_t> BittyBuzzLibMemberRegister::getValue() const {
    return m_value;
}
