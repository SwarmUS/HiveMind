#include "bittybuzz/BittyBuzzFactory.h"
#include "bittybuzz/BittyBuzzBytecode.h"
#include "bittybuzz/BittyBuzzUserFunctions.h"

extern "C" {
#include "main_bytecode.h"
}

BittyBuzzBytecode BittyBuzzFactory::createBittyBuzzBytecode(const ILogger& logger) {
    return BittyBuzzBytecode(logger, bcode, bcode_size);
}

std::array<FunctionRegister, 1> BittyBuzzFactory::createBittyBuzzFunctionRegisters() {
    return {{{BBZSTRID_logInt, BittyBuzzUserFunctions::logInt}}};
}