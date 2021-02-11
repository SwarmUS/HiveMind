#ifndef __MESSAGESENDER_H_
#define __MESSAGESENDER_H_

#include "IMessageSender.h"
#include <cpp-common/ICircularQueue.h>
#include <hivemind-host/HiveMindHostSerializer.h>
#include <logger/ILogger.h>

class MessageSender : IMessageSender {
  public:
    MessageSender(ICircularQueue<MessageDTO>& inputQueue,
                  IHiveMindHostSerializer& serializer,
                  ILogger& logger);

    ~MessageSender() override = default;

    bool processAndSerialize() override;

  private:
    ICircularQueue<MessageDTO>& m_inputQueue;
    IHiveMindHostSerializer& m_serializer;
    ILogger& m_logger;
};

#endif // __MESSAGESENDER_H_
