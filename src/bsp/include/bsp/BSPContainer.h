#ifndef _BSPFACTORY_H
#define _BSPFACTORY_H

#include "ICRC.h"
#include "bsp/IBSP.h"
#include "bsp/IHostUart.h"
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
     * @brief Returns an instance of the Host UART driver.
     */
    IHostUart& getHostUart();

    /**
     * @brief Returns an instance of the CRC driver.
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
} // namespace BSPContainer

#endif // _BSPFACTORY_H
