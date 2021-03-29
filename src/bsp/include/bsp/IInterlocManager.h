#ifndef __IINTERLOCMANAGER_H__
#define __IINTERLOCMANAGER_H__

#include "bsp/InterlocUpdate.h"
#include <functional>

class IInterlocManager {
  public:
    virtual ~IInterlocManager() = default;

    /**
     * @brief Dummy function to demonstrate working DW1000s
     */
    virtual void startInterloc() = 0;

    /**
     * @brief Registers a callback to be called when new interloc data is available
     * @param callback Callback to call
     */
    virtual void registerPositionUpdateCallback(std::function<void(InterlocUpdate)> callback) = 0;
};

#endif //__IINTERLOCMANAGER_H__
