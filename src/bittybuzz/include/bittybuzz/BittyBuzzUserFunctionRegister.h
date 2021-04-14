#ifndef __BITTYBUZZUSERFUNCTIONREGISTER_H_
#define __BITTYBUZZUSERFUNCTIONREGISTER_H_

#include <bbzvm.h>

class BittyBuzzUserFunctionRegister {
  public:
    BittyBuzzUserFunctionRegister(int16_t strId, bbzvm_funp functionPtr);
    int16_t getStringId() const;
    bbzvm_funp getFunctionPointer() const;

  private:
    int16_t m_strId;
    bbzvm_funp m_functionPtr;
    BittyBuzzUserFunctionRegister();
};

#endif // __BITTYBUZZUSERFUNCTIONREGISTER_H_
