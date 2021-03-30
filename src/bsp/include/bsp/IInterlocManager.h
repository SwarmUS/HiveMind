#ifndef __IINTERLOCMANAGER_H__
#define __IINTERLOCMANAGER_H__

#include "bsp/InterlocUpdate.h"
#include <functional>

typedef void (*positionUpdateCallbackFunction_t)(void* instance, InterlocUpdate update);

class IInterlocManager {
  public:
    virtual ~IInterlocManager() = default;

    /**
     * @brief Dummy function to demonstrate working DW1000s
     */
    virtual void startInterloc() = 0;

    /**
     * @brief Sets the callback to be called when new interloc data is available
     * @param callback Callback to call
     */
    virtual void setPositionUpdateCallback(positionUpdateCallbackFunction_t callback,
                                           void* context) = 0;
};

#endif //__IINTERLOCMANAGER_H__
