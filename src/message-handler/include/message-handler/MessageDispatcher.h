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
                      const uint16_t uuid,
                      ILogger& m_logger);

    ~MessageDispatcher() = default;

    bool deserializeAndDispatch();

  private:
    ICircularQueue<MessageDTO>& m_buzzOutputQueue;
    ICircularQueue<MessageDTO>& m_hostOutputQueue;
    ICircularQueue<MessageDTO>& m_remoteOutputQueue;
    IHiveMindHostDeserializer& m_deserializer;
    const uint16_t m_uuid;
    ILogger& m_logger;

    // handling funciton
    bool dispatchUserCall(const MessageDTO& message, const UserCallDestinationDTO& dest);
    bool dispatchUserCallRequest(const MessageDTO& message, const UserCallRequestDTO& request);
    bool dispatchUserCallResponse(const MessageDTO& message, const UserCallResponseDTO& response);
    bool dispatchRequest(const MessageDTO& message, const RequestDTO& request);
    bool dispatchResponse(const MessageDTO& message, const ResponseDTO& response);
    bool dispatchMessage(const MessageDTO& message);
};

#endif // __MESSAGEHANDLER_H_
