#include "bittybuzz/BittyBuzzFactory.h"
#include "BittyBuzzUserFunctions.h"
#include "bittybuzz/BittyBuzzBytecode.h"

extern "C" {
#include "main_bytecode.h"
}

BittyBuzzBytecode BittyBuzzFactory::createBittyBuzzBytecode(const ILogger& logger) {
    return BittyBuzzBytecode(logger, bcode, bcode_size);
}

std::array<FunctionRegister, 1> BittyBuzzFactory::createBittyBuzzFunctionRegisters() {
    return {{{BBZSTRID_logInt, bbz_user_functions::logInt}}};
}
