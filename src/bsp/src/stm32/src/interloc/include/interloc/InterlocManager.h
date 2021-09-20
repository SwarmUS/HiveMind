#ifndef __INTERLOCMANAGER_H__
#define __INTERLOCMANAGER_H__

#include "Decawave.h"
#include "DecawaveArray.h"
#include "InterlocStateHandler.h"
#include "UWBMessages.h"
#include <bsp/IInterlocManager.h>
#include <logger/ILogger.h>

class InterlocManager : public IInterlocManager {
  public:
    InterlocManager(ILogger& logger, InterlocStateHandler& stateHandler, DecawaveArray& decawaves);
    ~InterlocManager() override = default;

    void startInterloc() override;

    void setInterlocManagerState(InterlocStateDTO state) override;

    void configureTWRCalibration(uint16_t distanceCalibCm) override;

    void configureAngleCalibration(uint32_t numberOfFrames) override;

    void setInterlocManagerStateChangeCallback(
        interlocManagerStateChangeCallbackFunction_t callback, void* context) override;

    void setInterlocManagerRawAngleDataCallback(interlocRawAngleDataCallbackFunction_t callback,
                                                void* context) override;

    void setPositionUpdateCallback(positionUpdateCallbackFunction_t callback,
                                   void* context) override;

    void updateDistance(uint16_t robotId, float distance);

    void sendRawAngleData(BspInterlocRawAngleData& data);

    InterlocStateDTO getState() const;

    /**
     * Syncs the clocks of both DW1000s
     */
    void syncClocks();

  private:
    ILogger& m_logger;
    InterlocStateHandler& m_stateHandler;

    DecawaveArray& m_decawaves;

    uint16_t m_distanceCalibCm = 75;

    positionUpdateCallbackFunction_t m_positionUpdateCallback;
    void* m_positionUpdateCallbackContext;
    interlocManagerStateChangeCallbackFunction_t m_stateChangeCallback;
    void* m_stateChangeCallbackContext;
    interlocRawAngleDataCallbackFunction_t m_rawAngleDataCallback;
    void* m_rawAngleDataCallbackContext;

    InterlocStateDTO m_state;

    bool isFrameOk(UWBRxFrame frame);
    static uint8_t powerCorrection(double twrDistance);
};

#endif //__INTERLOCMANAGER_H__
