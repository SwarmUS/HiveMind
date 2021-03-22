#ifndef _BSPCONTAINER_H
#define _BSPCONTAINER_H

#include "ICRC.h"
#include "IInterlocManager.h"
#include "bsp/IBSP.h"
#include "bsp/ISpiEsp.h"
#include "bsp/IUSB.h"
#include "bsp/IUserInterface.h"

namespace BSPContainer {
    /**
     * @brief Returns an instance of the platform dependent BSP.
     */
    IBSP& getBSP();

    /**
     * @brief Returns an instance of the platform dependent UserInterface.
     */
    IUserInterface& getUserInterface();

    /**
     * @brief Returns an instance of the HardwareCRC driver.
     */
    ICRC& getCRC();

    /**
     * @brief Returns an instance of the Esp32 Spi driver.
     */
    ISpiEsp& getSpiEsp();

    /**
     * @brief Returns an instance of the USB CDC driver
     */
    IUSB& getUSB();

    /**
     * @brief Returns an instance of the interloc manager
     */
    IInterlocManager& getInterlocManager();
} // namespace BSPContainer

#endif // _BSPCONTAINER_H
