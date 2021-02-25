#include "bittybuzz/BittyBuzzSystem.h"

ILogger* BittyBuzzSystem::g_logger = NULL;
const IBittyBuzzStringResolver* BittyBuzzSystem::g_stringResolver = NULL;
IBittyBuzzFunctionRegister* BittyBuzzSystem::g_functionRegister = NULL;

void BittyBuzzSystem::functionCall(uint16_t stringId, uint16_t argc) {
    bbzvm_pushs(stringId);
    bbzheap_idx_t l = bbzvm_stack_at(0);
    bbzvm_pop();
    if (bbztable_get(vm->gsyms, l, &l) == 1) {
        bbzvm_pushnil(); // Push self table
        bbzvm_push(l);
        bbzvm_closure_call(argc);
        bbzvm_pop();
    }
}

void BittyBuzzSystem::errorReceiver(bbzvm_error errcode) {
    if (BittyBuzzSystem::g_logger != NULL) {
        BittyBuzzSystem::g_logger->log(
            LogLevel::Error,
            "BittyBuzz virtual machine error, pc: %d, stackptr: %d, state: %d error code: %d \n",
            vm->pc, vm->stackptr, vm->state, errcode);
    }
}
