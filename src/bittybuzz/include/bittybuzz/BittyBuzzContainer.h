#ifndef __BITTYBUZZCONTAINER_H_
#define __BITTYBUZZCONTAINER_H_

#include "BittyBuzzBytecode.h"
#include "BittyBuzzFunctionRegister.h"
#include "BittyBuzzMessageHandler.h"
#include "BittyBuzzStringResolver.h"
#include "BittyBuzzVm.h"
#include "IBittyBuzzVm.h"
#include <array>

namespace BittyBuzzContainer {

    /**
     *@brief return an instance of a bittybuzz message handler */
    BittyBuzzMessageHandler& getBBZMessageHandler();

    /**
     *@brief return an instance of a bittybuzz function register */
    BittyBuzzFunctionRegister& getBBZFunctionRegister();

} // namespace BittyBuzzContainer

#endif // __BITTYBUZZCONTAINER_H_
