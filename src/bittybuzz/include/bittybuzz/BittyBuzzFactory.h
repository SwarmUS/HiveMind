#ifndef __BITTYBUZZFACTORY_H_
#define __BITTYBUZZFACTORY_H_

#include "BittyBuzzBytecode.h"
#include "BittyBuzzStringResolver.h"
#include "IBittyBuzzVm.h"
#include <array>
#include <bittybuzz/BittyBuzzVm.h>

namespace BittyBuzzFactory {
    /**
     * @brief Creates a BittyBuzzBytecode with the main.bzz bytecode
     *
     * @param logger the logger to inject in the BittyBuzzBytecode
     *
     * */
    BittyBuzzBytecode createBittyBuzzBytecode(const ILogger& logger);

    /**
     * @brief Creates a BittyBuzzStringResolver with the main.bzz bytecode
     *
     * @param logger the logger to inject in the BittyBuzzStringResolver
     *
     * */
    BittyBuzzStringResolver createBittyBuzzStringResolver(const ILogger& logger);

    /**
     * @brief Creates an array of FunctionRegister associated with the code from main.bzz
     * */
    std::array<FunctionRegister, 2> createBittyBuzzFunctionRegisters();
} // namespace BittyBuzzFactory

#endif // __BITTYBUZZFACTORY_H_
