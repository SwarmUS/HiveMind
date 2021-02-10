#ifndef __MESSAGEHANDLER_H_
#define __MESSAGEHANDLER_H_

#include "IMessageDispatcher.h"
#include <cpp-common/ICircularQueue.h>
#include <hivemind-host/IHiveMindHostDeserializer.h>
#include <hivemind-host/MessageDTO.h>
#include <logger/ILogger.h>

class MessageDispatcher {
  public:
    MessageDispatcher(ICircularQueue<MessageDTO>& buzzOutputQ,
                      ICircularQueue<MessageDTO>& hostOutputQ,
                      ICircularQueue<MessageDTO>& remoteOutputQ,
                      IHiveMindHostDeserializer& deserializer,
                      ILogger& m_logger);

    ~MessageDispatcher() = default;

    bool deserializeAndDispatch();

  private:
    ICircularQueue<MessageDTO>& m_buzzOutputQueue;
    ICircularQueue<MessageDTO>& m_hostOutputQueue;
    ICircularQueue<MessageDTO>& m_remoteOutputQueue;
    IHiveMindHostDeserializer& m_deserializer;
    ILogger& m_logger;
};

#endif // __MESSAGEHANDLER_H_
