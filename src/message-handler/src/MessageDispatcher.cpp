#include "MessageDispatcher.h"
#include "message.pb.h"
#include <variant>

MessageDispatcher::MessageDispatcher(ICircularQueue<MessageDTO>& buzzOutputQ,
                                     ICircularQueue<MessageDTO>& hostOutputQ,
                                     ICircularQueue<MessageDTO>& remoteOutputQ,
                                     IHiveMindHostDeserializer& deserializer,
                                     IHiveMindApiRequestHandler& hivemindApiReqHandler,
                                     IGreetSender& greetSender,
                                     const IBSP& bsp,
                                     ILogger& logger) :
    m_buzzOutputQueue(buzzOutputQ),
    m_hostOutputQueue(hostOutputQ),
    m_remoteOutputQueue(remoteOutputQ),
    m_deserializer(deserializer),
    m_hivemindApiReqHandler(hivemindApiReqHandler),
    m_greetSender(greetSender),
    m_bsp(bsp),
    m_logger(logger) {}

bool MessageDispatcher::deserializeAndDispatch() {
    MessageDTO message;
    if (m_deserializer.deserializeFromStream(message)) {

        // Handle greet before filtering by id
        if (std::holds_alternative<GreetingDTO>(message.getMessage())) {
            return m_greetSender.sendGreet();
        }

        uint32_t destinationId = message.getDestinationId();
        if (destinationId == m_bsp.getUUId()) {
            return dispatchMessage(message);
        }

        // Message is for a remote host
        m_logger.log(LogLevel::Debug, "Message sent to remote queue");
        return m_remoteOutputQueue.push(message);
    }

    // Could not find the destination
    m_logger.log(LogLevel::Warn, "Fail to deserialize message");
    return false;
}

bool MessageDispatcher::dispatchUserCall(const MessageDTO& message, const UserCallTargetDTO& dest) {
    // Sending to the appropriate destination
    switch (dest) {
    case UserCallTargetDTO::BUZZ:
        m_logger.log(LogLevel::Debug, "Message sent to buzz queue");
        return m_buzzOutputQueue.push(message);
    case UserCallTargetDTO::HOST:
        m_logger.log(LogLevel::Debug, "Message sent to host queue");
        return m_hostOutputQueue.push(message);

    // Discard the message if unknown
    case UserCallTargetDTO::UNKNOWN:
    default:
        m_logger.log(LogLevel::Warn, "Unknown user call destination");
        return false;
    }
}

bool MessageDispatcher::dispatchUserCallRequest(const MessageDTO& message,
                                                const UserCallRequestDTO& request) {
    UserCallTargetDTO userCallDestination = request.getDestination();
    return dispatchUserCall(message, userCallDestination);
}

bool MessageDispatcher::dispatchUserCallResponse(const MessageDTO& message,
                                                 const UserCallResponseDTO& response) {
    UserCallTargetDTO userCallDestination = response.getDestination();
    return dispatchUserCall(message, userCallDestination);
}

bool MessageDispatcher::dispatchRequest(const MessageDTO& message, const RequestDTO& request) {

    const std::variant<std::monostate, UserCallRequestDTO, HiveMindApiRequestDTO,
                       SwarmApiRequestDTO>& variantReq = request.getRequest();

    if (const auto* uReq = std::get_if<UserCallRequestDTO>(&variantReq)) {
        return dispatchUserCallRequest(message, *uReq);
    }
    // Handle the response locally since it a hivemind api request
    if (const auto* hReq = std::get_if<HiveMindApiRequestDTO>(&variantReq)) {
        uint16_t uuid = m_bsp.getUUId();
        HiveMindApiResponseDTO hRes = m_hivemindApiReqHandler.handleRequest(*hReq);
        ResponseDTO resp(request.getId(), hRes);
        MessageDTO msg(message.getSourceId(), uuid, resp);

        if (message.getSourceId() == uuid) {
            return m_hostOutputQueue.push(msg);
        }
        if (message.getSourceId() != 0) {
            return m_remoteOutputQueue.push(msg);
        }
        return false;
    }
    if (std::holds_alternative<SwarmApiRequestDTO>(variantReq)) {
        m_logger.log(LogLevel::Warn, "Received swarm req on the hivemind");
    }
    return false;
}

bool MessageDispatcher::dispatchResponse(const MessageDTO& message, const ResponseDTO& response) {
    const std::variant<std::monostate, GenericResponseDTO, UserCallResponseDTO,
                       HiveMindApiResponseDTO, SwarmApiResponseDTO>& variantResp =
        response.getResponse();

    if (const auto* uResp = std::get_if<UserCallResponseDTO>(&variantResp)) {
        return dispatchUserCallResponse(message, *uResp);
    }
    if (std::holds_alternative<SwarmApiResponseDTO>(variantResp)) {
        m_logger.log(LogLevel::Warn, "Received swarm resp on the hivemind");
        return false;
    }

    // Either a HiveMindAPI or a unknown response, pipe it either way
    if (message.getDestinationId() == m_bsp.getUUId()) {
        return m_hostOutputQueue.push(message);
    }
    return m_remoteOutputQueue.push(message);
}

bool MessageDispatcher::dispatchMessage(const MessageDTO& message) {
    const std::variant<std::monostate, RequestDTO, ResponseDTO, GreetingDTO, BuzzMessageDTO>&
        variantMsg = message.getMessage();
    if (const auto* request = std::get_if<RequestDTO>(&variantMsg)) {
        return dispatchRequest(message, *request);
    }
    if (const auto* response = std::get_if<ResponseDTO>(&variantMsg)) {
        return dispatchResponse(message, *response);
    }
    if (std::holds_alternative<BuzzMessageDTO>(variantMsg)) {
        return m_buzzOutputQueue.push(message);
    }

    m_logger.log(LogLevel::Warn, "Unknown message, could not dispatch");
    return false;
}
