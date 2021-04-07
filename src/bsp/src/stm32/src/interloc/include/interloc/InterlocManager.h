#ifndef __INTERLOCMANAGER_H__
#define __INTERLOCMANAGER_H__

#include "Decawave.h"
#include "UWBMessages.h"
#include "interloc/InterlocStateHandler.h"
#include <bsp/IInterlocManager.h>
#include <logger/ILogger.h>

// TODO: Add to settings
#define PAN_ID 0x01

class InterlocManager : public IInterlocManager {
  public:
    InterlocManager(ILogger& logger, InterlocStateHandler& stateHandler);
    ~InterlocManager() override = default;

    void startInterloc() override;
    void startCalibSingleInitiator() override;
    void startCalibSingleResponder(uint16_t initiatorId,
                                   calibrationEndedCallbackFunction_t callback,
                                   void* context) override;
    void stopCalibration() override;
    void setCalibDistance(uint16_t distanceCalibCm) override;
    void setPositionUpdateCallback(positionUpdateCallbackFunction_t callback,
                                   void* context) override;

    /**
     * Syncs the clocks of both DW1000s
     */
    void syncClocks();

  private:
    ILogger& m_logger;
    InterlocStateHandler& m_stateHandler;

    Decawave m_decaA;
    Decawave m_decaB;

    uint8_t m_sequenceID = 0;
    uint16_t m_distanceCalibCm = 75;

    positionUpdateCallbackFunction_t m_positionUpdateCallback;
    calibrationEndedCallbackFunction_t m_calibrationEndedCallback;
    void* m_positionUpdateCallbackContext;
    void* m_calibrationEndedCallbackContext;
    uint16_t m_calibrationInitiatorId;

    void startDeviceCalibSingleInitiator(uint16_t destinationId, Decawave& device);
    void startDeviceCalibSingleResponder(uint16_t destinationId, Decawave& device);
    bool sendTWRSequence(uint16_t destinationId, Decawave& device);
    double receiveTWRSequence(uint16_t destinationId, Decawave& device);
    bool constructUWBHeader(uint16_t destinationId,
                            UWBMessages::FrameType frameType,
                            UWBMessages::FunctionCode functionCode,
                            uint8_t* buffer,
                            uint16_t bufferLength);
    bool isFrameOk(UWBRxFrame frame);
    static uint8_t powerCorrection(double twrDistance);
};

#endif //__INTERLOCMANAGER_H__
