#ifndef __BITTYBUZZCONTAINER_H_
#define __BITTYBUZZCONTAINER_H_

#include "BittyBuzzBytecode.h"
#include "BittyBuzzClosureRegister.h"
#include "BittyBuzzMessageHandler.h"
#include "BittyBuzzMessageService.h"
#include "BittyBuzzNeighborsManager.h"
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
    BittyBuzzClosureRegister& getBBZClosureRegister();

    /**
     *@brief return an instance of a bittybuzz message service*/
    BittyBuzzMessageService& getBBZMessageService();

    /**
     *@brief return an instance of a bittybuzz neighbors manager*/
    BittyBuzzNeighborsManager& getBBZNeighborsManager();

} // namespace BittyBuzzContainer

#endif // __BITTYBUZZCONTAINER_H_
