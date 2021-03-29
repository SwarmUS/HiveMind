#include "bittybuzz/BittyBuzzUserFunctions.h"
#include "bittybuzz/BittyBuzzSystem.h"
#include <LockGuard.h>

struct ForeachHostFContext {
    bool m_err = false;
    std::array<FunctionCallArgumentDTO, FunctionCallRequestDTO::FUNCTION_CALL_ARGUMENTS_MAX_LENGTH>
        m_arguments = {};
    uint16_t m_length = 0;
};

void foreachHostFCallback(bbzheap_idx_t key, bbzheap_idx_t value, void* params) {
    ForeachHostFContext* context = (ForeachHostFContext*)params;
    bbzobj_t* keyObj = bbzheap_obj_at(key); // NOLINT
    bbzobj_t* valueObj = bbzheap_obj_at(value); // NOLINT

    if (!bbztype_isint(*keyObj) || keyObj->i.value > (int16_t)context->m_arguments.size() ||
        keyObj->i.value < 0 || context->m_err) {
        context->m_err = true;
        return;
    }

    switch (bbztype(*valueObj)) {
    case BBZTYPE_INT:
        context->m_arguments[(uint8_t)keyObj->i.value] =
            FunctionCallArgumentDTO((int64_t)valueObj->i.value);
        context->m_length++;
        break;
    case BBZTYPE_FLOAT:
        context->m_arguments[(uint8_t)keyObj->i.value] =
            FunctionCallArgumentDTO(bbzfloat_tofloat(valueObj->f.value));
        context->m_length++;
        break;
    default:
        context->m_err = true;
    }
}

struct ArgNameAndType {
    bbzheap_idx_t m_key;
    bbzheap_idx_t m_value;
};

// We know that the size is 1, so the value will be written once
void foreachNameAndType(bbzheap_idx_t key, bbzheap_idx_t value, void* params) {
    ArgNameAndType* context = (ArgNameAndType*)params;
    context->m_key = key;
    context->m_value = value;
}

FunctionDescriptionArgumentTypeDTO getbbzObjType(bbzobj_t* bbzObj) {
    switch (bbztype(*bbzObj)) {
    case BBZTYPE_INT:
        return FunctionDescriptionArgumentTypeDTO::Int;
    case BBZTYPE_FLOAT:
        return FunctionDescriptionArgumentTypeDTO::Float;
    default:
        return FunctionDescriptionArgumentTypeDTO::Unknown;
    }
}

void BittyBuzzUserFunctions::log() {
    // Get the number of args
    uint16_t nArgs = bbzvm_locals_count();

    LockGuard lock(BittyBuzzSystem::g_ui->getPrintMutex());

    // Iterate through args, we start a arg 1 up to the nb of args
    for (uint16_t i = 1; i <= nArgs; i++) {

        bbzobj_t* obj = bbzheap_obj_at(bbzvm_locals_at(i)); // NOLINT

        switch (bbztype(*obj)) {
        case BBZTYPE_NIL: {
            BittyBuzzSystem::g_ui->print("[nil]");
            break;
        }
        case BBZTYPE_INT: {
            BittyBuzzSystem::g_ui->print("%d", obj->i.value);
            break;
        }
        case BBZTYPE_FLOAT: {
            BittyBuzzSystem::g_ui->print("%f", bbzfloat_tofloat(obj->f.value));
            break;
        }
        case BBZTYPE_TABLE: {
            BittyBuzzSystem::g_ui->print("[t:%d]", obj->t.value);
            break;
        }
        case BBZTYPE_STRING: {
            std::optional<const char*> optionString =
                BittyBuzzSystem::g_stringResolver->getString(obj->s.value);
            if (optionString) {
                BittyBuzzSystem::g_ui->print("%s", optionString.value());
            } else {
                BittyBuzzSystem::g_ui->print("{UNKNOWN STR}");
            }
            break;
        }
        case BBZTYPE_CLOSURE: {
            if (bbztype_isclosurelambda(*obj)) {
                BittyBuzzSystem::g_ui->print("[cl: %d}", obj->l.value.ref);
            } else {
                BittyBuzzSystem::g_ui->print("[c: %d]", (int)(intptr_t)obj->c.value);
            }
            break;
        }
        case BBZTYPE_USERDATA: {
            BittyBuzzSystem::g_ui->print("[u: %d]", (int)obj->u.value);
            break;
        }
        default: {
            printf("{UNKNOWN TYPE}");
            break;
        }
        }
    }
    BittyBuzzSystem::g_ui->flush();
    bbzvm_ret0();
}

void BittyBuzzUserFunctions::registerClosure() {
    bbzvm_assert_lnum(4); // NOLINT
    bbzobj_t* bbzFunctionName = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    bbzheap_idx_t bbzClosureHeapIdx = bbzvm_locals_at(2); // NOLINT
    bbzheap_idx_t bbzArgsDescHeapIdx = bbzvm_locals_at(3); // NOLINT
    bbzheap_idx_t bbzSelfHeapIdx = bbzvm_locals_at(4); // NOLINT

    bbzobj_t* bbzClosure = bbzheap_obj_at(bbzClosureHeapIdx); // NOLINT
    bbzobj_t* bbzArgsDesc = bbzheap_obj_at(bbzSelfHeapIdx); // NOLINT

    if (!bbztype_isstring(*bbzFunctionName) && !bbztype_isclosure(*bbzClosure) &&
        !bbztype_istable(*bbzArgsDesc)) {
        BittyBuzzSystem::g_logger->log(LogLevel::Error,
                                       "BBZ: Invalid type when registering function");

        bbzvm_seterror(BBZVM_ERROR_TYPE);
        return;
    }

    std::optional<const char*> functionNameOpt =
        BittyBuzzSystem::g_stringResolver->getString(bbzFunctionName->s.value);

    if (!functionNameOpt) {

        BittyBuzzSystem::g_logger->log(LogLevel::Error,
                                       "BBZ: String id not found when registering function");
        bbzvm_seterror(BBZVM_ERROR_STRING);
        return;
    }

    // Store the function name
    BittyBuzzFunctionDescription argsDescription(functionNameOpt.value());
    uint8_t argsSize = bbztable_size(bbzArgsDescHeapIdx);
    for (uint8_t i = 0; i < argsSize; i++) {

        bbzheap_idx_t key = bbzint_new(i);
        bbzheap_idx_t subTable;

        // Fetching the object in the table by index
        if (bbztable_get(bbzArgsDescHeapIdx, key, &subTable) == 0) {
            BittyBuzzSystem::g_logger->log(LogLevel::Error,
                                           "BBZ: Invalid args description on closure registration");
            bbzvm_seterror(BBZVM_ERROR_TYPE);
            return;
        }

        // Getting the subtable
        if (!bbztype_istable(*bbzheap_obj_at(subTable)) || bbztable_size(subTable) != 1) {
            BittyBuzzSystem::g_logger->log(LogLevel::Error,
                                           "BBZ: Invalid args description on closure registration");
            bbzvm_seterror(BBZVM_ERROR_TYPE);
            return;
        }

        ArgNameAndType keyValue;

        // We know that the subtable has a size of 1, so no overwrite on the foreach
        bbztable_foreach(subTable, foreachNameAndType, &keyValue);
        bbzobj_t* bbzArgName = bbzheap_obj_at(keyValue.m_key);
        bbzobj_t* bbzArgType = bbzheap_obj_at(keyValue.m_value);

        // Getting the string name
        std::optional<const char*> argNameOpt =
            BittyBuzzSystem::g_stringResolver->getString(bbzArgName->s.value);

        if (!argNameOpt) {
            BittyBuzzSystem::g_logger->log(LogLevel::Error,
                                           "BBZ: Arg name could not be found on registration");
            bbzvm_seterror(BBZVM_ERROR_STRING);
            return;
        }

        // Adding to the arg description
        const char* argName = argNameOpt.value();
        FunctionDescriptionArgumentTypeDTO argType = getbbzObjType(bbzArgType);
        argsDescription.addArgument(argName, argType);
    }

    bool ret = BittyBuzzSystem::g_closureRegister->registerClosure(
        functionNameOpt.value(), bbzClosureHeapIdx, bbzSelfHeapIdx, argsDescription);

    if (!ret) {
        BittyBuzzSystem::g_logger->log(LogLevel::Error, "BBZ: Could not register closure");
        bbzvm_seterror(BBZVM_ERROR_MEM);
        return;
    }

    bbzvm_ret0();
}

void BittyBuzzUserFunctions::callHostFunction() {
    bbzvm_assert_lnum(3); // NOLINT
    bbzobj_t* bbzId = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    bbzobj_t* bbzFunctionName = bbzheap_obj_at(bbzvm_locals_at(2)); // NOLINT
    bbzheap_idx_t bbzArgsTableHeapIdx = bbzvm_locals_at(3); // NOLINT

    ForeachHostFContext context;
    std::optional<const char*> functionName =
        BittyBuzzSystem::g_stringResolver->getString(bbzFunctionName->s.value);

    bbztable_foreach(bbzArgsTableHeapIdx, foreachHostFCallback, &context);

    if (context.m_err) {
        BittyBuzzSystem::g_logger->log(LogLevel::Error,
                                       "BBZ: Error parsing argument list, host FCall");
        bbzvm_seterror(BBZVM_ERROR_TYPE);
        return;
    }

    bool ret = BittyBuzzSystem::g_messageService->callHostFunction(
        (uint16_t)bbzId->i.value, functionName.value(), context.m_arguments.data(),
        context.m_length);
    if (!ret) {
        BittyBuzzSystem::g_logger->log(LogLevel::Warn, "BBZ: could not call host FCall");
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
