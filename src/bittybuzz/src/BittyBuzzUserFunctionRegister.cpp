#include "BittyBuzzUserFunctionRegister.h"

BittyBuzzUserFunctionRegister::BittyBuzzUserFunctionRegister(int16_t strId,
                                                             bbzvm_funp functionPtr) :
    m_strId(strId), m_functionPtr(functionPtr) {}

int16_t BittyBuzzUserFunctionRegister::getStringId() const { return m_strId; }

bbzvm_funp BittyBuzzUserFunctionRegister::getFunctionPointer() const { return m_functionPtr; }
