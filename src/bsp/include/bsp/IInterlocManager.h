#ifndef __IINTERLOCMANAGER_H__
#define __IINTERLOCMANAGER_H__

#include "BspInterlocAngleRawData.h"
#include "bsp/InterlocUpdate.h"
#include <functional>
#include <pheromones/interloc/ConfigureAngleParametersDTO.h>
#include <pheromones/interloc/InterlocStateDTO.h>

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

    /**
     * @brief Sets the number of frames to accumulate when in angle calibration mode
     * @param numberOfFrames Number of frames to accumulate
     */
    virtual void configureAngleCalibration(uint32_t numberOfFrames) = 0;

    /**
     * Sets the callback to call when a state change occurs in the interloc manager
     * @param callback Callback
     * @param context Context to pass the callback
     */
    virtual void setInterlocManagerStateChangeCallback(
        interlocManagerStateChangeCallbackFunction_t callback, void* context) = 0;

    /**
     * Sets the callback to call when a raw angle data is sent back to the PC
     * @param callback Callback
     * @param context Context to pass the callback
     */
    virtual void setInterlocManagerRawAngleDataCallback(
        interlocRawAngleDataCallbackFunction_t callback, void* context) = 0;

    /**
     * Updates the parameters used to calculate an angle
     * @param newParams DTO to save as new parameters
     */
    virtual void updateAngleCalculatorParameters(const ConfigureAngleParametersDTO& newParams) = 0;
};

#endif //__IINTERLOCMANAGER_H__
