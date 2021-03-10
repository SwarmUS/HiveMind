
#ifndef __BITTYBUZZVM_TPP_
#define __BITTYBUZZVM_TPP_

#include "bittybuzz/BittyBuzzSystem.h"
#include "bittybuzz/BittyBuzzVm.h"
#include <bbzvm.h>

template <typename Container>
BittyBuzzVm::BittyBuzzVm(const IBittyBuzzBytecode& bytecode,
                         const IBittyBuzzStringResolver& stringResolver,
                         IBittyBuzzMessageHandler& messageHandler,
                         IBittyBuzzClosureRegister& closureRegister,
                         IBittyBuzzMessageService& messageService,
                         const IBSP& bsp,
                         ILogger& logger,
                         IUserInterface& ui,
                         const Container& container) :
    m_bytecode(bytecode), m_bsp(bsp), m_messageHandler(messageHandler), m_logger(logger), m_ui(ui) {
    // Init global variable
    vm = &m_bbzVm;
    BittyBuzzSystem::g_logger = &logger;
    BittyBuzzSystem::g_ui = &ui;
    BittyBuzzSystem::g_stringResolver = &stringResolver;
    BittyBuzzSystem::g_closureRegister = &closureRegister;
    BittyBuzzSystem::g_messageService = &messageService;

    // Init vm
    bbzvm_construct(m_bsp.getUUId());
    bbzvm_set_error_receiver(BittyBuzzSystem::errorReceiver);
    bbzvm_set_bcode(m_bytecode.getBytecodeFetchFunction(), m_bytecode.getBytecodeLength());
    bbzringbuf_construct(&m_bbzPayloadBuff, m_bbzMsgBuff.data(), 1, m_bbzMsgBuff.size());

    // Function registration
    for (UserFunctionRegister functionRegister : container) {
        bbzvm_function_register(functionRegister.m_strId, functionRegister.m_functionPtr);
    }

    vm->state = BBZVM_STATE_READY;
    // TODO: Fix variable declaration not called when not in init
    BittyBuzzSystem::functionCall(__BBZSTRID_init);
}

#endif // __BITTYBUZZVM_TPP_
