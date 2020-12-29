#include "BittyBuzzUserFunctions.h"
#include "BittyBuzzSystem.h"

void bbz_user_functions::logNumber() {
    bbzvm_assert_lnum(1); // NOLINT
    bbzobj_t* int_val = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT

    bbz_system::logger->log(LogLevel::Info, "Int value: %d \n", int_val);
    bbzvm_ret0();
}
