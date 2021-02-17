#include "bittybuzz/BittyBuzzVm.h"
#include "bittybuzz/BittyBuzzSystem.h"
#include <bbzvm.h>

FunctionRegister::FunctionRegister(uint8_t strId, bbzvm_funp functionPtr) :
    m_strId(strId), m_functionPtr(functionPtr) {}

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
