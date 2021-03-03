#include "MessageHandler.h"

MessageHandler::MessageHandler(ICircularQueue<MessageDTO>& hostOutputQ,
                               ICircularQueue<MessageDTO>& remoteOutputQ,
                               const IBSP& bsp,
                               ILogger& logger) :
    m_hostOutputQueue(hostOutputQ),
    m_remoteOutputQueue(remoteOutputQ),
    m_bsp(bsp),
    m_logger(logger) {}

bool MessageHandler::handleMessage(const MessageDTO& message) {
    (void)message;
    return true;
}
