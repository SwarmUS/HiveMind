#include "bittybuzz/BittyBuzzVm.h"
#include "bittybuzz/BittyBuzzSystem.h"
#include <bbzvm.h>

FunctionRegister::FunctionRegister(uint8_t strId, bbzvm_funp functionPtr) :
    m_strId(strId), m_functionPtr(functionPtr) {}

bool BittyBuzzVm::step() {

    if (vm->state != BBZVM_STATE_ERROR) {
        bbzvm_process_inmsgs();
        BittyBuzzSystem::functionCall(__BBZSTRID_step, 0);
        bbzvm_process_outmsgs();
        for (uint16_t i = 0; i < m_messageHandler.messageQueueLength(); i++) {
            if (!m_messageHandler.processMessage()) {
                m_logger.log(LogLevel::Warn,
                             "BBVM: Could not process message or the queue is full");
            }
        }
        return true;
    }

    return false;
}

bbzvm_state BittyBuzzVm::getSate() const { return vm->state; }
bbzvm_error BittyBuzzVm::getError() const { return vm->error; }
