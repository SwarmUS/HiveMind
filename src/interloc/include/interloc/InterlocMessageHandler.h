#ifndef __INTERLOCMESSAGEHANDLER_H__
#define __INTERLOCMESSAGEHANDLER_H__

#include "IInterlocMessageHandler.h"
#include <bsp/IBSP.h>
#include <bsp/IInterlocManager.h>
#include <cpp-common/ICircularQueue.h>
#include <hivemind-host/InterlocAPIDTO.h>
#include <hivemind-host/MessageDTO.h>
#include <logger/ILogger.h>

class InterlocMessageHandler : public IInterlocMessageHandler {
  public:
    InterlocMessageHandler(ILogger& logger,
                           IInterlocManager& interlocManager,
                           IBSP& bsp,
                           ICircularQueue<MessageDTO>& inputQueue);

    bool processMessage();

  private:
    ILogger& m_logger;
    IInterlocManager& m_interlocManager;
    IBSP& m_bsp;
    ICircularQueue<MessageDTO>& m_inputQueue;

    bool handleMessage(const MessageDTO& dto) const;
    static bool handleCalibrationMessage(const CalibrationMessageDTO& dto);
};

#endif //__INTERLOCMESSAGEHANDLER_H__
