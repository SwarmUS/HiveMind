#include "MessageDispatcher.h"
#include <bsp/SettingsContainer.h>

MessageDispatcher::MessageDispatcher(ICircularQueue<MessageDTO>& buzzOutputQ,
                                     ICircularQueue<MessageDTO>& hostOutputQ,
                                     ICircularQueue<MessageDTO>& remoteOutputQ,
                                     IHiveMindHostDeserializer& deserializer,
                                     ILogger& logger) :
    m_buzzOutputQueue(buzzOutputQ),
    m_hostOutputQueue(hostOutputQ),
    m_remoteOutputQueue(remoteOutputQ),
    m_deserializer(deserializer),
    m_logger(logger) {}

bool MessageDispatcher::dispatchUserCall(MessageDTO& message, UserCallDestinationDTO& dest) {
    // Sending to the appropriate destination
    switch (dest) {
    case UserCallDestinationDTO::BUZZ:
        m_logger.log(LogLevel::Debug, "Message sent to buzz queue");
        m_buzzOutputQueue.push(message);
        return true;
    case UserCallDestinationDTO::HOST:
        m_logger.log(LogLevel::Debug, "Message sent to host queue");
        m_hostOutputQueue.push(message);
        return true;

    // Discard the message if unknown
    case UserCallDestinationDTO::UNKNOWN:
    default:
        m_logger.log(LogLevel::Warn, "Unknown user call destination");
        return false;
    }
}

bool MessageDispatcher::dispatchUserCallRequest(MessageDTO& message, UserCallRequestDTO& request) {
    UserCallDestinationDTO userCallDestination = request.getDestination();
    dispatchUserCall(message, request);
}

bool MessageDispatcher::dispatchUserCallResponse(MessageDTO& message,
                                                 UserCallResponseDTO& response) {
    UserCallDestinationDTO userCallDestination = response.getResponse();
    dispatchUserCall(message, request);
}

bool MessageDispatcher::dispatchRequest(MessageDTO& message, RequestDTO& request) {

    const std::variant<std::monostate, UserCallRequestDTO>& variantReq = request.getRequest();

    if (const UserCallRequestDTO* uReq = std::get_if<UserCallRequestDTO>(&variantReq)) {
        return dispatchUserCallRequest(*uReq);
    }
    return false;
}

bool MessageDispatcher::dispatchResponse(MessageDTO& message, ResponseDTO& response) {
    const std::variant<std::monostate, UserCallResponseDTO>& variantResp = response.getResponse();

    if (const UserCallResponseDTO* uResp = std::get_if<UserCallResponseDTO>(&variantResp)) {
        return dispatchUserCallResponse(message, *uResp);
    }
    return false;
}

bool MessageDispatcher::dispatchMessage(MessageDTO& message) {
    const std::variant<std::monostate, RequestDTO, ResponseDTO>& variantMsg = message.getMessage();
    if (const RequestDTO* request = std::get_if<RequestDTO>(&variantMsg)) {
        return dispatchRequest(message, *request);
    }
    if (const ResponseDTO* response = std::get_if<ResponseDTO>(&variantMsg)) {
        return dispatchResponse(message, *response)
    }
    return false;
}

bool MessageDispatcher::deserializeAndDispatch() {
    MessageDTO message;
    if (m_deserializer.deserializeFromStream(message)) {

        // Message is for local use
        if (message.getDestinationId() == SettingsContainer::getUUID()) {
            const std::variant<std::monostate, RequestDTO, ResponseDTO>& = message.getMessage();

        }

        // Message is for a remote host
        else {
            m_logger.log(LogLevel::Debug, "Message sent to remote queue");
            m_remoteOutputQueue.push(message);
            return true;
        }
    }

    // Could not find the destination
    m_logger.log(LogLevel::Warn, "Destination could not be found");
    return false;
}
