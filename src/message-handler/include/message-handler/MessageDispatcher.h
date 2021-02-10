#ifndef __MESSAGEHANDLER_H_
#define __MESSAGEHANDLER_H_

#include "IMessageDispatcher.h"
#include <cpp-common/ICircularQueue.h>
#include <hivemind-host/IHiveMindHostDeserializer.h>
#include <hivemind-host/MessageDTO.h>

class MessageDispatcher {
  public:
    MessageDispatcher(ICircularQueue<MessageDTO>& buzzOutputQ,
                      ICircularQueue<MessageDTO>& tcpOutputQ,
                      ICircularQueue<MessageDTO>& uartOutputQ,
                      IHiveMindHostDeserializer& deserializer);

    ~MessageDispatcher() = default;

    bool deserializeAndDispatch();

  private:
    ICircularQueue<MessageDTO>& m_buzzOutputQueue;
    ICircularQueue<MessageDTO>& m_tcpOutputQueue;
    ICircularQueue<MessageDTO>& m_uartOutputQueue;
    IHiveMindHostDeserializer& m_deserializer;
};

#endif // __MESSAGEHANDLER_H_
