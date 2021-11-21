#ifndef __INTERLOCMESSAGEHANDLER_H__
#define __INTERLOCMESSAGEHANDLER_H__

#include "IInterlocMessageHandler.h"
#include <INotificationQueue.h>
#include <bsp/IBSP.h>
#include <bsp/IInterlocManager.h>
#include <cpp-common/ICircularQueue.h>
#include <logger/ILogger.h>
#include <pheromones/MessageDTO.h>
#include <pheromones/interloc/InterlocAPIDTO.h>

class InterlocMessageHandler : public IInterlocMessageHandler {
  public:
    InterlocMessageHandler(ILogger& logger,
                           IInterlocManager& interlocManager,
                           IBSP& bsp,
                           ICircularQueue<MessageDTO>& inputQueue,
                           ICircularQueue<MessageDTO>& hostQueue,
                           ICircularQueue<MessageDTO>& remoteQueue,
                           INotificationQueue<InterlocUpdate>& interlocPositionUpdateQueue);

    virtual ~InterlocMessageHandler() = default;

    bool processMessage() override;

    bool getDumpEnabled() const override;

    bool sendInterlocDump(InterlocUpdate* updatesHistory, uint8_t updatesLength) override;

  private:
    ILogger& m_logger;
    IInterlocManager& m_interlocManager;
    IBSP& m_bsp;
    ICircularQueue<MessageDTO>& m_inputQueue;
    ICircularQueue<MessageDTO>& m_hostQueue;
    ICircularQueue<MessageDTO>& m_remoteQueue;
    INotificationQueue<InterlocUpdate>& m_interlocPositionUpdateQueue;

    std::array<GetNeighborResponseDTO, InterlocDumpDTO::MAX_UPDATES_SIZE> m_updateDtoArray;

    uint16_t m_messageSourceId;
    bool m_dumpsEnabled;

    bool handleMessage(const MessageDTO& dto);
    bool handleStateChangeMessage(const SetInterlocStateDTO dto) const;
    bool handleConfigurationMessage(const InterlocConfigurationDTO& dto);

    // bool handleCalibrationMessage(const CalibrationMessageDTO& dto, uint16_t sourceId) const;
    ICircularQueue<MessageDTO>& getQueueForDestination(uint16_t destinationId) const;

    void stateChangeCallback(InterlocStateDTO previousState, InterlocStateDTO newState);
    static void stateChangeCallbackStatic(void* context,
                                          InterlocStateDTO previousState,
                                          InterlocStateDTO newState);

    void rawAngleDataCallback(BspInterlocRawAngleData& data);
    static void rawAngleDataCallbackStatic(void* context, BspInterlocRawAngleData& data);

    void ensureSendMessage(MessageDTO& msg);
    static InterlocRawAngleDataDTO constructRawDataMessage(BspInterlocRawAngleData& data,
                                                           uint32_t fromId,
                                                           uint8_t numFrames);
};

#endif //__INTERLOCMESSAGEHANDLER_H__
