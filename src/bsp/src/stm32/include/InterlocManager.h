#ifndef __INTERLOCMANAGER_H__
#define __INTERLOCMANAGER_H__

#include "Decawave.h"
#include "UWBMessages.h"
#include <bsp/IInterlocManager.h>
#include <logger/ILogger.h>

// TODO: Add to settings
#define PAN_ID 0x01

class InterlocManager : public IInterlocManager {
  public:
    explicit InterlocManager(ILogger& logger);
    ~InterlocManager() override = default;

    void startInterloc() override;
    void startCalibSingleInitiator() override;
    void startCalibSingleResponder() override;
    void setCalibDistance(uint16_t distanceCalibCm) override;
    void setCalibFinishedCallback(void (*fct)(void* context), void* context) override;
    void setPositionUpdateCallback(positionUpdateCallbackFunction_t callback,
                                   void* context) override;

  private:
    ILogger& m_logger;
    Decawave m_decaA;
    Decawave m_decaB;

    uint8_t m_sequenceID = 0;
    uint16_t m_distanceCalibCm = 75;
    void (*m_calibFinishedCallback)(void* context);
    void* m_calibFinishedCallbackContext;
    void startDeviceCalibSingleInitiator(uint16_t destinationId, Decawave& device);
    void startDeviceCalibSingleResponder(uint16_t destinationId, Decawave& device);
    bool sendTWRSequence(uint16_t destinationId, Decawave& device);
    double receiveTWRSequence(uint16_t destinationId, Decawave& device);
    bool constructUWBHeader(uint16_t destinationId,
                            UWBMessages::FrameType frameType,
                            UWBMessages::FunctionCode functionCode,
                            uint8_t* buffer,
                            uint16_t bufferLength);

    positionUpdateCallbackFunction_t m_positionUpdateCallback;
    void* m_positionUpdateCallbackContext;
    bool isFrameOk(UWBRxFrame frame);
};

#endif //__INTERLOCMANAGER_H__
