#ifndef __BITTYBUZZFACTORY_H_
#define __BITTYBUZZFACTORY_H_

#include "bittybuzz/BittyBuzzBytecode.h"
#include "bittybuzz/IBittyBuzzVm.h"
#include <array>
#include <bittybuzz/BittyBuzzVm.h>

namespace BittyBuzzFactory {
    /**
     * @brief Creates a BittyBuzzBytecode with the main.bzz bytecode
     * */
    BittyBuzzBytecode createBittyBuzzBytecode(const ILogger& logger);

    /**
     * @brief Creates an array of FunctionRegister associated with the code from main.bzz
     * */
    std::array<FunctionRegister, 1> createBittyBuzzFunctionRegisters();
} // namespace BittyBuzzFactory

#endif // __BITTYBUZZFACTORY_H_
