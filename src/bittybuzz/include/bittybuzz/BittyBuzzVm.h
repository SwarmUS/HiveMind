#ifndef __BITTYBUZZVM_H_
#define __BITTYBUZZVM_H_

#include "IBittyBuzzBytecode.h"
#include "IBittyBuzzStringResolver.h"
#include "IBittyBuzzVm.h"
#include <array>
#include <bsp/IBSP.h>
#include <logger/ILogger.h>

#define BBZ_MSG_BUFF_SIZE 16

class FunctionRegister {
  public:
    FunctionRegister(uint8_t strId, bbzvm_funp functionPtr);
    uint8_t m_strId;
    bbzvm_funp m_functionPtr;

  private:
    FunctionRegister();
};

class BittyBuzzVm : public IBittyBuzzVm {
  public:
    /**
     *@brief The constructor of the bbvm
     *@param bytecode the bytecode that the vm will run
     *@param stringResolver the string resolver used in the VM
     *@param bsp a reference to the bsp
     *@param logger a reference to a logger
     *@param container the provided iterator
     *@tparam Container an iterator of any sort (stl container) that returns a FunctionRegister*/
    template <typename Container>
    BittyBuzzVm(const IBittyBuzzBytecode& bytecode,
                const IBittyBuzzStringResolver& stringResolver,
                const IBSP& bsp,
                ILogger& logger,
                const Container& container);

    ~BittyBuzzVm() override = default;

    bool step() override;

    bbzvm_state getSate() const override;

    bbzvm_error getError() const override;

  private:
    const IBittyBuzzBytecode& m_bytecode;
    const IBSP& m_bsp;
    ILogger& m_logger;

    bbzvm_t m_bbzVm;
    std::array<uint8_t, BBZ_MSG_BUFF_SIZE> m_bbzMsgBuff;
    bbzmsg_payload_t m_bbzPayloadBuff;
};

#include "BittyBuzzVm.tpp"

#endif // __BITTYBUZZVM_H_
