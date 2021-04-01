#ifndef __IINTERLOCMANAGER_H__
#define __IINTERLOCMANAGER_H__

#include "bsp/InterlocUpdate.h"
#include <functional>

typedef void (*positionUpdateCallbackFunction_t)(void* instance, InterlocUpdate update);
typedef void (*calibrationEndedCallbackFunction_t)(void* instance, uint16_t initiatorId);

class IInterlocManager {
  public:
    virtual ~IInterlocManager() = default;

    /**
     * @brief Dummy function to demonstrate working DW1000s
     */
    virtual void startInterloc() = 0;

    /**
     * @brief Start calibration in initiator mode, it polls in TWR and send the final msg
     */
    virtual void startCalibSingleInitiator() = 0;

    /**
     * @brief Start calibration in responder mode, it sends the response msg and compute the
     * distance
     * @param initiatorId The ID of the HiveBoard from which the start command came from (used to
     * send a response to the correct board)
     * @param callback Callback to call when calibration is finished
     * @param context The context to pass to the callback
     */
    virtual void startCalibSingleResponder(uint16_t initiatorId,
                                           calibrationEndedCallbackFunction_t callback,
                                           void* context) = 0;

    /**
     * @brief Stops an ongoing calibration
     */
    virtual void stopCalibration() = 0;

    /**
     * @brief Sets the targeted distance for calibration
     * @param distanceCalibCm Distance between the devices in calibration mode
     */
    virtual void setCalibDistance(uint16_t distanceCalibCm) = 0;

    /**
     * @brief Sets the callback to be called when new interloc data is available
     * @param callback Callback to call
     * @param context The context to pass to the callback
     */
    virtual void setPositionUpdateCallback(positionUpdateCallbackFunction_t callback,
                                           void* context) = 0;
};

#endif //__IINTERLOCMANAGER_H__
