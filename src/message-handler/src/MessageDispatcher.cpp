#include "MessageDispatcher.h"

MessageDispatcher::MessageDispatcher(ICircularQueue<MessageDTO>& buzzOutputQ,
                                     ICircularQueue<MessageDTO>& hostOutputQ,
                                     ICircularQueue<MessageDTO>& remoteOutputQ,
                                     IHiveMindHostDeserializer& deserializer,
                                     uint16_t bspUuid,
                                     ILogger& logger) :
    m_buzzOutputQueue(buzzOutputQ),
    m_hostOutputQueue(hostOutputQ),
    m_remoteOutputQueue(remoteOutputQ),
    m_deserializer(deserializer),
    m_bspUuid(bspUuid),
    m_logger(logger) {}

bool MessageDispatcher::deserializeAndDispatch() {
    MessageDTO message;
    if (m_deserializer.deserializeFromStream(message)) {

        uint32_t destinationId = message.getDestinationId();
        if (destinationId == m_bspUuid) {
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

    const std::variant<std::monostate, UserCallRequestDTO>& variantReq = request.getRequest();

    if (const auto* uReq = std::get_if<UserCallRequestDTO>(&variantReq)) {
        return dispatchUserCallRequest(message, *uReq);
    }
    return false;
}

bool MessageDispatcher::dispatchResponse(const MessageDTO& message, const ResponseDTO& response) {
    const std::variant<std::monostate, GenericResponseDTO, UserCallResponseDTO>& variantResp =
        response.getResponse();

    if (const auto* uResp = std::get_if<UserCallResponseDTO>(&variantResp)) {
        return dispatchUserCallResponse(message, *uResp);
    }
    // Unknow response, just pipe it to the host since it won't be recognized by buzz since it uses
    // the same lib, note that generic response is included here. Buzz don't need a generic response
    if (message.getDestinationId() == m_bspUuid) {
        return m_hostOutputQueue.push(message);
    }
    // Not recognized and not for here, send it to remote
    return m_remoteOutputQueue.push(message);
}

bool MessageDispatcher::dispatchMessage(const MessageDTO& message) {
    const std::variant<std::monostate, RequestDTO, ResponseDTO>& variantMsg = message.getMessage();
    if (const auto* request = std::get_if<RequestDTO>(&variantMsg)) {
        return dispatchRequest(message, *request);
    }
    if (const auto* response = std::get_if<ResponseDTO>(&variantMsg)) {
        return dispatchResponse(message, *response);
    }
    return false;
}
