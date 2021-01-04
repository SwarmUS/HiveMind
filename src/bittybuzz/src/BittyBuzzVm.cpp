#include "bittybuzz/BittyBuzzVm.h"
#include "bittybuzz/BittyBuzzSystem.h"
#include <bbzvm.h>

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
