
#ifndef __BITTYBUZZVM_TPP_
#define __BITTYBUZZVM_TPP_

#include "bittybuzz/BittyBuzzSystem.h"
#include "bittybuzz/BittyBuzzVm.h"
#include <bbzvm.h>

template <typename Container>
BittyBuzzVm::BittyBuzzVm(const IBittyBuzzBytecode& bytecode,
                         const IBittyBuzzStringResolver& stringResolver,
                         IBittyBuzzMessageHandler& messageHandler,
                         IBittyBuzzFunctionRegister& functionRegister,
                         const IBSP& bsp,
                         ILogger& logger,
                         const Container& container) :
    m_bytecode(bytecode), m_bsp(bsp), m_messageHandler(messageHandler), m_logger(logger) {
    // Init global variable
    vm = &m_bbzVm;
    BittyBuzzSystem::g_logger = &logger;
    BittyBuzzSystem::g_stringResolver = &stringResolver;
    BittyBuzzSystem::g_functionRegister = &functionRegister;

    // Init vm
    bbzvm_construct(m_bsp.getUUId());
    bbzvm_set_error_receiver(BittyBuzzSystem::errorReceiver);
    bbzvm_set_bcode(m_bytecode.getBytecodeFetchFunction(), m_bytecode.getBytecodeLength());
    bbzringbuf_construct(&m_bbzPayloadBuff, m_bbzMsgBuff.data(), 1, m_bbzMsgBuff.size());

    // Function registration
    for (FunctionRegister functionRegister : container) {
        bbzvm_function_register(functionRegister.m_strId, functionRegister.m_functionPtr);
    }

    vm->state = BBZVM_STATE_READY;
    BittyBuzzSystem::functionCall(__BBZSTRID_init, 0);
}

#endif // __BITTYBUZZVM_TPP_
