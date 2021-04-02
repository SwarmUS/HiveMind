#ifndef __INTERLOCMESSAGEHANDLER_H__
#define __INTERLOCMESSAGEHANDLER_H__

#include "IInterlocMessageHandler.h"
#include <bsp/IBSP.h>
#include <bsp/IInterlocManager.h>
#include <cpp-common/ICircularQueue.h>
#include <logger/ILogger.h>
#include <pheromones/InterlocAPIDTO.h>
#include <pheromones/MessageDTO.h>

class InterlocMessageHandler : public IInterlocMessageHandler {
  public:
    InterlocMessageHandler(ILogger& logger,
                           IInterlocManager& interlocManager,
                           IBSP& bsp,
                           ICircularQueue<MessageDTO>& inputQueue,
                           ICircularQueue<MessageDTO>& hostQueue,
                           ICircularQueue<MessageDTO>& remoteQueue);

    virtual ~InterlocMessageHandler() = default;

    bool processMessage() override;

  private:
    ILogger& m_logger;
    IInterlocManager& m_interlocManager;
    IBSP& m_bsp;
    ICircularQueue<MessageDTO>& m_inputQueue;
    ICircularQueue<MessageDTO>& m_hostQueue;
    ICircularQueue<MessageDTO>& m_remoteQueue;

    bool handleMessage(const MessageDTO& dto) const;
    bool handleCalibrationMessage(const CalibrationMessageDTO& dto, uint16_t sourceId) const;
    ICircularQueue<MessageDTO>& getQueueForDestination(uint16_t destinationId) const;

    void notifyCalibrationEnded(uint16_t initiatorId);
    static void notifyCalibrationEndedStatic(void* context, uint16_t initiatorId);
};

#endif //__INTERLOCMESSAGEHANDLER_H__
