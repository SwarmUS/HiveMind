#include "bittybuzz/BittyBuzzUserFunctions.h"
#include "bittybuzz/BittyBuzzSystem.h"

void BittyBuzzUserFunctions::logInt() {
    bbzvm_assert_lnum(1); // NOLINT
    bbzobj_t* int_val = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT

    BittyBuzzSystem::logger->log(LogLevel::Info, "Int value: %d", int_val->i.value);
    bbzvm_ret0();
}
