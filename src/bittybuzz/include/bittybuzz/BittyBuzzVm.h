#ifndef __BITTYBUZZVM_H_
#define __BITTYBUZZVM_H_

#include "bittybuzz/IBittyBuzzBytecode.h"
#include "bittybuzz/IBittyBuzzVm.h"
#include "bsp/IBSP.h"
#include "logger/ILogger.h"
#include <array>

typedef struct {
    uint8_t strId;
    bbzvm_funp functionPtr;

} FunctionRegister;

class BittyBuzzVm : public IBittyBuzzVm {
  public:
    BittyBuzzVm(const IBittyBuzzBytecode& bytecode,
                const IBSP& bsp,
                const ILogger& logger,
                const FunctionRegister* functionRegisters,
                uint16_t lengthFunctionRegisters);

    ~BittyBuzzVm() override = default;

    bool step() override;

    bbzvm_state getSate() const override;

    bbzvm_error getError() const override;

  private:
    const IBittyBuzzBytecode& m_bytecode;
    const IBSP& m_bsp;
    const ILogger& m_logger;

    bbzvm_t m_bbzVm;
    uint8_t m_bbzMsgBuff[11];
    bbzmsg_payload_t m_bbzPayloadBuff;
};

#endif // __BITTYBUZZVM_H_
