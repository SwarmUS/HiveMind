#ifndef _BSPFACTORY_H
#define _BSPFACTORY_H

#include "bsp/IBSP.h"
#include "bsp/IPhoneCommunication.h"
#include "bsp/IUserInterface.h"

namespace BSPContainer {
    /**
     * @brief Return an instance of the platform dependent BSP.
     */
    IBSP& getBSP();
    IPhoneCommunication& getPhoneCommunication();
} // namespace BSPContainer

#endif // _BSPFACTORY_H
