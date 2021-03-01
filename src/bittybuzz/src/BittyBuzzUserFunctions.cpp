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
        BittyBuzzSystem::g_logger->log(LogLevel::Warn, "BBZ: Wrong argument type to logString");
    }

    std::optional<const char*> optionString =
        BittyBuzzSystem::g_stringResolver->getString(bbzString->s.value);

    if (optionString) {
        BittyBuzzSystem::g_logger->log(LogLevel::Info, "BBZ: %s", optionString.value());
    } else {
        BittyBuzzSystem::g_logger->log(LogLevel::Warn, "BBZ: String id not found");
    }

    bbzvm_ret0();
}

void BittyBuzzUserFunctions::registerClosure() {
    bbzvm_assert_lnum(2); // NOLINT
    bbzobj_t* bbzFunctionName = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    bbzheap_idx_t bbzClosureHeapIdx = bbzvm_locals_at(2); // NOLINT
    bbzobj_t* bbzClosure = bbzheap_obj_at(bbzClosureHeapIdx); // NOLINT

    if (!bbztype_isstring(*bbzFunctionName) && !bbztype_isclosure(*bbzClosure)) {
        BittyBuzzSystem::g_logger->log(LogLevel::Info,
                                       "BBZ: invalid type when registering function");
        return;
    }

    std::optional<const char*> optionString =
        BittyBuzzSystem::g_stringResolver->getString(bbzFunctionName->s.value);

    if (optionString) {
        // Store the function name
        // TODO: add support for table with arg name as key and
        bool ret = BittyBuzzSystem::g_closureRegister->registerClosure(optionString.value(),
                                                                       bbzClosureHeapIdx);
        if (!ret) {
            BittyBuzzSystem::g_logger->log(LogLevel::Warn, "BBZ: Could not register closure");
        }

    } else {

        BittyBuzzSystem::g_logger->log(LogLevel::Warn,
                                       "BBZ: String id not found when registering function");
    }
}

void BittyBuzzUserFunctions::callHostFunction() {
    bbzvm_assert_lnum(3); // NOLINT
    bbzobj_t* bbzId = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    bbzobj_t* bbzFunctionName = bbzheap_obj_at(bbzvm_locals_at(2)); // NOLINT
    // bbzobj_t* bbzArgsTable = bbzheap_obj_at(bbzvm_locals_at(3)); // NOLINT

    std::optional<const char*> functionName =
        BittyBuzzSystem::g_stringResolver->getString(bbzFunctionName->s.value);

    BittyBuzzSystem::g_messageService->callFunction((uint16_t)bbzId->i.value, functionName.value(),
                                                    NULL, 0);
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
