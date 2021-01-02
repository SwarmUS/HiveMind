#include "bittybuzz/BittyBuzzVm.h"
#include "bittybuzz/BittyBuzzSystem.h"
#include <FreeRTOS.h>
#include <bbzvm.h>
#include <task.h>

BittyBuzzVm::BittyBuzzVm(const IBittyBuzzBytecode& bytecode,
                         const IBSP& bsp,
                         const ILogger& logger,
                         const FunctionRegister* functionRegisters,
                         uint16_t lengthFunctionRegisters) :

    m_bytecode(bytecode), m_bsp(bsp), m_logger(logger) {
    // Init global variable
    vm = &m_bbzVm;
    BittyBuzzSystem::logger = &logger;

    // Init vm
    bbzvm_construct(m_bsp.getUUId());
    bbzvm_set_error_receiver(BittyBuzzSystem::errorReceiver);
    bbzvm_set_bcode(m_bytecode.getBytecodeFetchFunction(), m_bytecode.getBytecodeLength());
    bbzringbuf_construct(&m_bbzPayloadBuff, m_bbzMsgBuff, 1, 11);

    // Function registration
    for (uint16_t i = 0; i < lengthFunctionRegisters; i++) {
        bbzvm_function_register(functionRegisters[i].strId, functionRegisters[i].functionPtr);
    }

    vm->state = BBZVM_STATE_READY;
    BittyBuzzSystem::functionCall(__BBZSTRID_init);
}

bool BittyBuzzVm::step() {

    if (vm->state != BBZVM_STATE_ERROR) {
        bbzvm_process_inmsgs();
        BittyBuzzSystem::functionCall(__BBZSTRID_step);
        bbzvm_process_outmsgs();
        return true;
    }

    return false;
}

bbzvm_state BittyBuzzVm::getSate() const { return vm->state; }
bbzvm_error BittyBuzzVm::getError() const { return vm->error; }
