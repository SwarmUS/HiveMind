#ifndef __BITTYBUZZFACTORY_H_
#define __BITTYBUZZFACTORY_H_

#include "BittyBuzzBytecode.h"
#include "BittyBuzzLib.h"
#include "BittyBuzzLibMemberRegister.h"
#include "BittyBuzzStringResolver.h"
#include "IBittyBuzzVm.h"
#include <array>
#include <bittybuzz/BittyBuzzVm.h>

namespace BittyBuzzFactory {
    /** @brief Creates a BittyBuzzBytecode with the main.bzz bytecode
     * @param logger the logger to inject in the BittyBuzzBytecode */
    BittyBuzzBytecode createBittyBuzzBytecode(ILogger& logger);

    /** @brief Creates a BittyBuzzStringResolver with the main.bzz bytecode
     * @param logger the logger to inject in the BittyBuzzStringResolver */
    BittyBuzzStringResolver createBittyBuzzStringResolver(ILogger& logger);

    /** @brief Creates an array of FunctionRegister associated with the code from main.bzz */
    BittyBuzzLib<std::array<BittyBuzzLibMemberRegister, 15>> createBittyBuzzGlobalLib(
        int16_t vmStepDelay);

    /** @brief Creates a the math table library for the bvm */
    BittyBuzzLib<std::array<BittyBuzzLibMemberRegister, 24>> createBittyBuzzMathLib();

    /** @brief Creates a the ui table library for the bvm */
    BittyBuzzLib<std::array<BittyBuzzLibMemberRegister, 2>> createBittyBuzzUILib();

} // namespace BittyBuzzFactory

#endif // __BITTYBUZZFACTORY_H_
