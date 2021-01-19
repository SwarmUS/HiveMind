#ifndef _BSPFACTORY_H
#define _BSPFACTORY_H

#include "ICRC.h"
#include "bsp/IBSP.h"
#include "bsp/IHostUart.h"
#include "bsp/IUserInterface.h"

namespace BSPContainer {
    /**
     * @brief Return an instance of the platform dependent BSP.
     */
    IBSP& getBSP();

    /**
     * @brief Return an instance of the platform dependent UserInterface.
     */
    IUserInterface& getUserInterface();

    IHostUart& getHostUart();

    ICRC& getCRC();

} // namespace BSPContainer

#endif // _BSPFACTORY_H
