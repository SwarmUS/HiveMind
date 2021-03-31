#ifndef __IINTERLOCMANAGER_H__
#define __IINTERLOCMANAGER_H__

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
    virtual void startCalibSingleInit() = 0;

    /**
     * @brief Start calibration in responder mode, it sends the response msg and compute the distance
     */
    virtual void startCalibSingleRespond() = 0;

    /**
     * @brief Sets the targeted distance for calibration
     */
    virtual void setCalibDistance(uint16_t distanceForCalibCm) = 0;

    /**
     * @brief Sets the function to call when the calibration ends
     */
    virtual void setCalibFinishedCallback(void (*fct)(void* context), void* context) = 0;

};

#endif //__IINTERLOCMANAGER_H__
