#include "bittybuzz/BittyBuzzUserFunctions.h"
#include "bittybuzz/BittyBuzzSystem.h"

void BittyBuzzUserFunctions::logInt() {
    bbzvm_assert_lnum(1); // NOLINT
    bbzobj_t* intVal = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT

    BittyBuzzSystem::logger->log(LogLevel::Info, "Int value: %d", intVal->i.value);
    bbzvm_ret0();
}

void BittyBuzzUserFunctions::logString() {
    bbzvm_assert_lnum(1); // NOLINT
    bbzobj_t* string = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT

    if (bbztype_isstring(*string) != 1) {
        BittyBuzzSystem::logger->log(LogLevel::Warn, "BittyBuzz: Wrong argument type to logString");
    }

    std::optional<const char*> optionChar =
        BittyBuzzSystem::stringResolver->getString(string->s.value);

    if (optionChar) {
        BittyBuzzSystem::logger->log(LogLevel::Info, "BittyBuzz: %s", optionChar.value());
    } else {
        BittyBuzzSystem::logger->log(LogLevel::Warn, "BittyBuzz: String id not found");
    }

    bbzvm_ret0();
}
