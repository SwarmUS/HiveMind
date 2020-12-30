#include "BittyBuzzSystem.h"

const ILogger* bbz_system::logger = NULL;

void bbz_system::functionCall(uint16_t strid) {
    bbzvm_pushs(strid);
    bbzheap_idx_t l = bbzvm_stack_at(0);
    bbzvm_pop();
    if (bbztable_get(vm->gsyms, l, &l)) {
        bbzvm_pushnil(); // Push self table
        bbzvm_push(l);
        bbzvm_closure_call(0);
        bbzvm_pop();
    }
}

void bbz_system::errorReceiver(bbzvm_error errcode) {
    if (bbz_system::logger != NULL) {
        bbz_system::logger->log(
            LogLevel::Error,
            "BittyBuzz virtual machine error, pc: %d, stackptr: %d, state: %d error code: %d \n",
            vm->pc, vm->stackptr, vm->state, errcode);
    }
}
