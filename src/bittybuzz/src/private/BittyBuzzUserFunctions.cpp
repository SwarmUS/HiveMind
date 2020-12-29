#include "BittyBuzzUserFunctions.h"
#include "BittyBuzzSystem.h"

void bbz_user_functions::logInt() {
    bbzvm_assert_lnum(1); // NOLINT
    bbzobj_t* int_val = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT

    bbz_system::logger->log(LogLevel::Info, "Int value: %d", int_val->mdata);
    bbzvm_ret0();
}
