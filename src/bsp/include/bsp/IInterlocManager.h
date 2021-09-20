#ifndef __IINTERLOCMANAGER_H__
#define __IINTERLOCMANAGER_H__

#include "BspInterlocAngleRawData.h"
#include "bsp/InterlocUpdate.h"
#include <functional>
#include <pheromones/interloc/InterlocStateDTO.h>

typedef void (*positionUpdateCallbackFunction_t)(void* instance, InterlocUpdate update);
typedef void (*interlocManagerStateChangeCallbackFunction_t)(void* instance,
                                                             InterlocStateDTO previousState,
                                                             InterlocStateDTO newState);
typedef void (*interlocRawAngleDataCallbackFunction_t)(void* instance,
                                                       BspInterlocRawAngleData& rawAngleData);

class IInterlocManager {
  public:
    virtual ~IInterlocManager() = default;

    /**
     * @brief Dummy function to demonstrate working DW1000s
     */
    virtual void startInterloc() = 0;

    virtual void setInterlocManagerState(InterlocStateDTO state) = 0;

    /**
     * @brief Sets the targeted distance for calibration
     * @param distanceCalibCm Distance between the devices in calibration mode
     */
    virtual void configureTWRCalibration(uint16_t distanceCalibCm) = 0;

    virtual void configureAngleCalibration(uint32_t numberOfFrames) = 0;

    virtual void setInterlocManagerStateChangeCallback(
        interlocManagerStateChangeCallbackFunction_t callback, void* context) = 0;

    virtual void setInterlocManagerRawAngleDataCallback(
        interlocRawAngleDataCallbackFunction_t callback, void* context) = 0;

    /**
     * @brief Sets the callback to be called when new interloc data is available
     * @param callback Callback to call
     * @param context The context to pass to the callback
     */
    virtual void setPositionUpdateCallback(positionUpdateCallbackFunction_t callback,
                                           void* context) = 0;
};

#endif //__IINTERLOCMANAGER_H__
