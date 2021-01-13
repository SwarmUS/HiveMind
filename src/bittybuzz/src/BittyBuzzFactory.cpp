#include "bittybuzz/BittyBuzzFactory.h"
#include "bittybuzz/BittyBuzzUserFunctions.h"

extern "C" {
#include <main_bytecode.h>
#include <main_string.h>
}

BittyBuzzBytecode BittyBuzzFactory::createBittyBuzzBytecode(const ILogger& logger) {
    return BittyBuzzBytecode(logger, bcode, bcode_size);
}

BittyBuzzStringResolver BittyBuzzFactory::createBittyBuzzStringResolver(const ILogger& logger) {
    return BittyBuzzStringResolver(g_bbzStringResolverArray.data(), g_bbzStringResolverArray.size(),
                                   BBZSTRING_OFFSET, logger);
}

std::array<FunctionRegister, 2> BittyBuzzFactory::createBittyBuzzFunctionRegisters() {
    return {{{BBZSTRID_log, BittyBuzzUserFunctions::logString},
             {BBZSTRID_logInt, BittyBuzzUserFunctions::logInt}}};
}
