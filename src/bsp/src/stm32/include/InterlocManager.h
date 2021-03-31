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
    void startCalibSingleInit(uint16_t destinationId, Decawave& device);
    void startCalibSingleRespond(uint16_t destinationId, Decawave& device);


  private:
    ILogger& m_logger;
    Decawave m_decaA;
    Decawave m_decaB;

    uint8_t m_sequenceID = 0;
    bool sendTWRSequence(uint16_t destinationId, Decawave& device);
    double receiveTWRSequence(uint16_t destinationId, Decawave& device);

    bool constructUWBHeader(uint16_t destinationId,
                            UWBMessages::FrameType frameType,
                            UWBMessages::FunctionCode functionCode,
                            uint8_t* buffer,
                            uint16_t bufferLength);
};

#endif //__INTERLOCMANAGER_H__
