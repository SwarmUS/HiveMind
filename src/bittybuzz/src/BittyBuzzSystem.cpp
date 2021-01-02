#include "bittybuzz/BittyBuzzSystem.h"

const ILogger* BittyBuzzSystem::logger = NULL;

void BittyBuzzSystem::functionCall(uint16_t strid) {
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

void BittyBuzzSystem::errorReceiver(bbzvm_error errcode) {
    if (BittyBuzzSystem::logger != NULL) {
        BittyBuzzSystem::logger->log(
            LogLevel::Error,
            "BittyBuzz virtual machine error, pc: %d, stackptr: %d, state: %d error code: %d \n",
            vm->pc, vm->stackptr, vm->state, errcode);
    }
}
