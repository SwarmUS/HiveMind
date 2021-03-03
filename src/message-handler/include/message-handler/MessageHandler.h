#ifndef __MESSAGEHANDLER_H_
#define __MESSAGEHANDLER_H_

#include "IMessageHandler.h"
#include <bsp/IBSP.h>
#include <cpp-common/ICircularQueue.h>
#include <logger/ILogger.h>

class MessageHandler : public IMessageHandler {
  public:
    MessageHandler(ICircularQueue<MessageDTO>& hostOutputQ,
                   ICircularQueue<MessageDTO>& remoteOutputQ,
                   const IBSP& bsp,
                   ILogger& logger);

    ~MessageHandler() override = default;

    bool handleMessage(const MessageDTO& message) override;

  private:
    ICircularQueue<MessageDTO>& m_hostOutputQueue;
    ICircularQueue<MessageDTO>& m_remoteOutputQueue;
    const IBSP& m_bsp;
    ILogger& m_logger;
};

#endif // __MESSAGEHANDLER_H_
