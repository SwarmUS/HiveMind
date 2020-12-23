#ifndef _BSPFACTORY_H
#define _BSPFACTORY_H

#include "bsp/IBSP.h"
#include "bsp/IUserInterface.h"

class BSPFactory {
  public:
    BSPFactory() = default;
    ~BSPFactory() = default;

    static IBSP* getBSP();
};

#endif // _BSPFACTORY_H
