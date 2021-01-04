
#ifndef __BITTYBUZZVM_TPP_
#define __BITTYBUZZVM_TPP_

#include "bittybuzz/BittyBuzzSystem.h"
#include "bittybuzz/BittyBuzzVm.h"
#include <bbzvm.h>

template <typename Container>
BittyBuzzVm::BittyBuzzVm(const IBittyBuzzBytecode& bytecode,
                         const IBSP& bsp,
                         const ILogger& logger,
                         const Container& container) :
    m_bytecode(bytecode), m_bsp(bsp), m_logger(logger) {
    // Init global variable
    vm = &m_bbzVm;
    BittyBuzzSystem::logger = &logger;

    // Init vm
    bbzvm_construct(m_bsp.getUUId());
    bbzvm_set_error_receiver(BittyBuzzSystem::errorReceiver);
    bbzvm_set_bcode(m_bytecode.getBytecodeFetchFunction(), m_bytecode.getBytecodeLength());
    bbzringbuf_construct(&m_bbzPayloadBuff, m_bbzMsgBuff.data(), 1, m_bbzMsgBuff.size());

    // Function registration
    for (FunctionRegister functionRegister : container) {
        bbzvm_function_register(functionRegister.strId, functionRegister.functionPtr);
    }

    vm->state = BBZVM_STATE_READY;
    BittyBuzzSystem::functionCall(__BBZSTRID_init);
}

#endif // __BITTYBUZZVM_TPP_
