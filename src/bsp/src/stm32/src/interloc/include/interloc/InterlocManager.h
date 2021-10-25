#ifndef __INTERLOCMANAGER_H__
#define __INTERLOCMANAGER_H__

#include "Decawave.h"
#include "DecawaveArray.h"
#include "InterlocStateHandler.h"
#include "UWBMessages.h"
#include <INotificationQueue.h>
#include <application-interface/IButtonCallbackRegister.h>
#include <bsp/IInterlocManager.h>
#include <logger/ILogger.h>

class InterlocManager : public IInterlocManager {
  public:
    InterlocManager(ILogger& logger,
                    InterlocStateHandler& stateHandler,
                    DecawaveArray& decawaves,
                    INotificationQueue<InterlocUpdate>& interlocUpdateQueue,
                    IButtonCallbackRegister& buttonCallbackRegister);
    ~InterlocManager() override = default;

    void startInterloc() override;

    void setInterlocManagerState(InterlocStateDTO state) override;

    void configureTWRCalibration(uint16_t distanceCalibCm) override;

    void configureAngleCalibration(uint32_t numberOfFrames) override;

    void setInterlocManagerStateChangeCallback(
        interlocManagerStateChangeCallbackFunction_t callback, void* context) override;

    void setInterlocManagerRawAngleDataCallback(interlocRawAngleDataCallbackFunction_t callback,
                                                void* context) override;

    void updateAngleCalculatorParameters(const ConfigureAngleParametersDTO& newParams) override;

    void updateInterloc(uint16_t robotId,
                        std::optional<float> distance,
                        std::optional<float> angle);

    void sendRawAngleData(BspInterlocRawAngleData& data);

    InterlocStateDTO getState() const;

    /**
     * Syncs the clocks of both DW1000s
     */
    void syncClocks();

    void buttonCallback();
    static void staticButtonCallback(void* context);

  private:
    ILogger& m_logger;
    InterlocStateHandler& m_stateHandler;
    IButtonCallbackRegister& m_buttonCallbackRegister;

    DecawaveArray& m_decawaves;

    uint16_t m_distanceCalibCm = 75;

    INotificationQueue<InterlocUpdate>& m_interlocUpdateQueue;

    interlocManagerStateChangeCallbackFunction_t m_stateChangeCallback;
    void* m_stateChangeCallbackContext;
    interlocRawAngleDataCallbackFunction_t m_rawAngleDataCallback;
    void* m_rawAngleDataCallbackContext;

    InterlocStateDTO m_state;

    static uint8_t powerCorrection(double twrDistance);
};

#endif //__INTERLOCMANAGER_H__
