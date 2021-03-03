#ifndef __MESSAGEDISPATCHER_H_
#define __MESSAGEDISPATCHER_H_

#include "IHiveMindApiRequestHandler.h"
#include "IMessageDispatcher.h"
#include <bsp/IBSP.h>
#include <cpp-common/ICircularQueue.h>
#include <hivemind-host/IHiveMindHostDeserializer.h>
#include <hivemind-host/MessageDTO.h>
#include <logger/ILogger.h>

class MessageDispatcher : IMessageDispatcher {
  public:
    MessageDispatcher(ICircularQueue<MessageDTO>& buzzOutputQ,
                      ICircularQueue<MessageDTO>& hostOutputQ,
                      ICircularQueue<MessageDTO>& remoteOutputQ,
                      IHiveMindHostDeserializer& deserializer,
                      IHiveMindApiRequestHandler& hivemindApiReqHandler,
                      const IBSP& bsp,
                      ILogger& logger);

    ~MessageDispatcher() override = default;

    bool deserializeAndDispatch() override;

  private:
    ICircularQueue<MessageDTO>& m_buzzOutputQueue;
    ICircularQueue<MessageDTO>& m_hostOutputQueue;
    ICircularQueue<MessageDTO>& m_remoteOutputQueue;
    IHiveMindHostDeserializer& m_deserializer;
    IHiveMindApiRequestHandler& m_hivemindApiReqHandler;
    const IBSP& m_bsp;
    ILogger& m_logger;

    // handling funciton
    bool dispatchUserCall(const MessageDTO& message, const UserCallTargetDTO& dest);
    bool dispatchUserCallRequest(const MessageDTO& message, const UserCallRequestDTO& request);
    bool dispatchUserCallResponse(const MessageDTO& message, const UserCallResponseDTO& response);
    bool dispatchRequest(const MessageDTO& message, const RequestDTO& request);
    bool dispatchResponse(const MessageDTO& message, const ResponseDTO& response);
    bool dispatchMessage(const MessageDTO& message);
};

#endif // __MESSAGEDISPATCHER_H_
