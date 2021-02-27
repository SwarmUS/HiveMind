#include "bittybuzz/BittyBuzzFactory.h"
#include "bittybuzz/BittyBuzzUserFunctions.h"

extern "C" {
#include <main_bytecode.h>
#include <main_string.h>
}

BittyBuzzBytecode BittyBuzzFactory::createBittyBuzzBytecode(ILogger& logger) {
    return BittyBuzzBytecode(logger, bcode, bcode_size);
}

BittyBuzzStringResolver BittyBuzzFactory::createBittyBuzzStringResolver(ILogger& logger) {
    return BittyBuzzStringResolver(g_bbzStringResolverArray.data(), g_bbzStringResolverArray.size(),
                                   BBZSTRING_OFFSET, logger);
}

std::array<UserFunctionRegister, 10> BittyBuzzFactory::createBittyBuzzFunctionRegisters() {
    return {{
        {BBZSTRID_log, BittyBuzzUserFunctions::logString},
        {BBZSTRID_logInt, BittyBuzzUserFunctions::logInt},
        {BBZSTRID_is_nil, BittyBuzzUserFunctions::isNil},
        {BBZSTRID_is_int, BittyBuzzUserFunctions::isInt},
        {BBZSTRID_is_float, BittyBuzzUserFunctions::isFloat},
        {BBZSTRID_is_string, BittyBuzzUserFunctions::isString},
        {BBZSTRID_is_table, BittyBuzzUserFunctions::isTable},
        {BBZSTRID_is_closure, BittyBuzzUserFunctions::isClosure},
        {BBZSTRID_is_lambda_closure, BittyBuzzUserFunctions::isLambdaClosure},
        {BBZSTRID_register_function, BittyBuzzUserFunctions::registerFuntion},
    }};
}
