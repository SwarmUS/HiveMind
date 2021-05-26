#ifndef __MESSAGEDISPATCHER_H_
#define __MESSAGEDISPATCHER_H_

#include "IGreetSender.h"
#include "IHiveConnectHiveMindApiMessageHandler.h"
#include "IHiveMindHostApiRequestHandler.h"
#include "IMessageDispatcher.h"
#include <bsp/IBSP.h>
#include <cpp-common/ICircularQueue.h>
#include <logger/ILogger.h>
#include <pheromones/IHiveMindHostDeserializer.h>
#include <pheromones/MessageDTO.h>

class MessageDispatcher : IMessageDispatcher {
  public:
    MessageDispatcher(ICircularQueue<MessageDTO>& buzzOutputQ,
                      ICircularQueue<MessageDTO>& hostOutputQ,
                      ICircularQueue<MessageDTO>& remoteOutputQ,
                      ICircularQueue<MessageDTO>& interlocQ,
                      IHiveMindHostDeserializer& deserializer,
                      IHiveMindHostApiRequestHandler& hivemindApiReqHandler,
                      IHiveConnectHiveMindApiMessageHandler& hiveconnectApiMessageHandler,
                      IGreetSender& greetSender,
                      const IBSP& bsp,
                      ILogger& logger);

    ~MessageDispatcher() override = default;

    bool deserializeAndDispatch() override;

  private:
    ICircularQueue<MessageDTO>& m_buzzOutputQueue;
    ICircularQueue<MessageDTO>& m_hostOutputQueue;
    ICircularQueue<MessageDTO>& m_remoteOutputQueue;
    ICircularQueue<MessageDTO>& m_interlocQueue;
    IHiveMindHostDeserializer& m_deserializer;
    IHiveMindHostApiRequestHandler& m_hivemindApiReqHandler;
    IHiveConnectHiveMindApiMessageHandler& m_hiveconnectApiMessageHandler;
    IGreetSender& m_greetSender;
    const IBSP& m_bsp;
    ILogger& m_logger;

    // handling funciton
    bool dispatchUserCall(const MessageDTO& message, const UserCallTargetDTO& dest);
    bool dispatchUserCallRequest(const MessageDTO& message, const UserCallRequestDTO& request);
    bool dispatchUserCallResponse(const MessageDTO& message, const UserCallResponseDTO& response);
    bool dispatchRequest(const MessageDTO& message, const RequestDTO& request);
    bool dispatchResponse(const MessageDTO& message, const ResponseDTO& response);
    bool dispatchBuzzMessage(const MessageDTO& message);
    bool dispatchMessage(const MessageDTO& message);
};

#endif // __MESSAGEDISPATCHER_H_
