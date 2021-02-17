#include "bittybuzz/BittyBuzzUserFunctions.h"
#include "bittybuzz/BittyBuzzSystem.h"

void BittyBuzzUserFunctions::logInt() {
    bbzvm_assert_lnum(1); // NOLINT
    bbzobj_t* intVal = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT

    BittyBuzzSystem::g_logger->log(LogLevel::Info, "Int value: %d", intVal->i.value);
    bbzvm_ret0();
}

void BittyBuzzUserFunctions::logString() {
    bbzvm_assert_lnum(1); // NOLINT
    bbzobj_t* bbzString = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT

    if (bbztype_isstring(*bbzString) != 1) {
        BittyBuzzSystem::g_logger->log(LogLevel::Warn,
                                       "BittyBuzz: Wrong argument type to logString");
    }

    std::optional<const char*> optionString =
        BittyBuzzSystem::g_stringResolver->getString(bbzString->s.value);

    if (optionString) {
        BittyBuzzSystem::g_logger->log(LogLevel::Info, "BittyBuzz: %s", optionString.value());
    } else {
        BittyBuzzSystem::g_logger->log(LogLevel::Warn, "BittyBuzz: String id not found");
    }

    bbzvm_ret0();
}

void BittyBuzzUserFunctions::isNil() {
    bbzvm_assert_lnum(1); // NOLINT
    bbzobj_t* bbzObj = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT

    bbzvm_pushi(bbztype_isnil(*bbzObj));

    bbzvm_ret1();
}

void BittyBuzzUserFunctions::isInt() {
    bbzvm_assert_lnum(1); // NOLINT
    bbzobj_t* bbzObj = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT

    bbzvm_pushi(bbztype_isint(*bbzObj));

    bbzvm_ret1();
}

void BittyBuzzUserFunctions::isFloat() {
    bbzvm_assert_lnum(1); // NOLINT
    bbzobj_t* bbzObj = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT

    bbzvm_pushi(bbztype_isfloat(*bbzObj));

    bbzvm_ret1();
}

void BittyBuzzUserFunctions::isString() {
    bbzvm_assert_lnum(1); // NOLINT
    bbzobj_t* bbzObj = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT

    bbzvm_pushi(bbztype_isstring(*bbzObj));

    bbzvm_ret1();
}

void BittyBuzzUserFunctions::isTable() {
    bbzvm_assert_lnum(1); // NOLINT
    bbzobj_t* bbzObj = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT

    bbzvm_pushi(bbztype_istable(*bbzObj));

    bbzvm_ret1();
}

void BittyBuzzUserFunctions::isClosure() {
    bbzvm_assert_lnum(1); // NOLINT
    bbzobj_t* bbzObj = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT

    bbzvm_pushi(bbztype_isclosure(*bbzObj));

    bbzvm_ret1();
}

void BittyBuzzUserFunctions::isLambdaClosure() {
    bbzvm_assert_lnum(1); // NOLINT
    bbzobj_t* bbzObj = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT

    bbzvm_pushi(bbztype_isclosurelambda(*bbzObj));

    bbzvm_ret1();
}
