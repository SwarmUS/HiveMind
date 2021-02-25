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

std::array<FunctionRegister, 10> BittyBuzzFactory::createBittyBuzzFunctionRegisters() {
    return {{
        {BBZSTRID_log, BittyBuzzUserFunctions::logString},
        {BBZSTRID_logInt, BittyBuzzUserFunctions::logInt},
        {BBZSTRID_isNil, BittyBuzzUserFunctions::isNil},
        {BBZSTRID_isInt, BittyBuzzUserFunctions::isInt},
        {BBZSTRID_isFloat, BittyBuzzUserFunctions::isFloat},
        {BBZSTRID_isString, BittyBuzzUserFunctions::isString},
        {BBZSTRID_isTable, BittyBuzzUserFunctions::isTable},
        {BBZSTRID_isClosure, BittyBuzzUserFunctions::isClosure},
        {BBZSTRID_isLambdaClosure, BittyBuzzUserFunctions::isLambdaClosure},
        {BBZSTRID_registerFunction, BittyBuzzUserFunctions::registerFuntion},
    }};
}
