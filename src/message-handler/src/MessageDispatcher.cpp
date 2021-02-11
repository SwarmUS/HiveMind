#include "MessageDispatcher.h"
#include <bsp/SettingsContainer.h>

MessageDispatcher::MessageDispatcher(ICircularQueue<MessageDTO>& buzzOutputQ,
                                     ICircularQueue<MessageDTO>& hostOutputQ,
                                     ICircularQueue<MessageDTO>& remoteOutputQ,
                                     IHiveMindHostDeserializer& deserializer,
                                     uint16_t uuid,
                                     ILogger& logger) :
    m_buzzOutputQueue(buzzOutputQ),
    m_hostOutputQueue(hostOutputQ),
    m_remoteOutputQueue(remoteOutputQ),
    m_deserializer(deserializer),
    m_uuid(uuid),
    m_logger(logger) {}

bool MessageDispatcher::deserializeAndDispatch() {
    MessageDTO message;
    if (m_deserializer.deserializeFromStream(message)) {

        uint32_t destinationId = message.getDestinationId();
        if (destinationId == m_uuid) {
            return dispatchMessage(message);
        }

        // Message is for a remote host
        m_logger.log(LogLevel::Debug, "Message sent to remote queue");
        m_remoteOutputQueue.push(message);
        return true;
    }

    // Could not find the destination
    m_logger.log(LogLevel::Warn, "Destination could not be found");
    return false;
}

bool MessageDispatcher::dispatchUserCall(const MessageDTO& message,
                                         const UserCallDestinationDTO& dest) {
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

bool MessageDispatcher::dispatchUserCallRequest(const MessageDTO& message,
                                                const UserCallRequestDTO& request) {
    UserCallDestinationDTO userCallDestination = request.getDestination();
    return dispatchUserCall(message, userCallDestination);
}

bool MessageDispatcher::dispatchUserCallResponse(const MessageDTO& message,
                                                 const UserCallResponseDTO& response) {
    UserCallDestinationDTO userCallDestination = response.getDestination();
    return dispatchUserCall(message, userCallDestination);
}

bool MessageDispatcher::dispatchRequest(const MessageDTO& message, const RequestDTO& request) {

    const std::variant<std::monostate, UserCallRequestDTO>& variantReq = request.getRequest();

    if (const UserCallRequestDTO* uReq = std::get_if<UserCallRequestDTO>(&variantReq)) {
        return dispatchUserCallRequest(message, *uReq);
    }
    return false;
}

bool MessageDispatcher::dispatchResponse(const MessageDTO& message, const ResponseDTO& response) {
    const std::variant<std::monostate, UserCallResponseDTO>& variantResp = response.getResponse();

    if (const UserCallResponseDTO* uResp = std::get_if<UserCallResponseDTO>(&variantResp)) {
        return dispatchUserCallResponse(message, *uResp);
    }
    return false;
}

bool MessageDispatcher::dispatchMessage(const MessageDTO& message) {
    const std::variant<std::monostate, RequestDTO, ResponseDTO>& variantMsg = message.getMessage();
    if (const RequestDTO* request = std::get_if<RequestDTO>(&variantMsg)) {
        return dispatchRequest(message, *request);
    }
    if (const ResponseDTO* response = std::get_if<ResponseDTO>(&variantMsg)) {
        return dispatchResponse(message, *response);
    }
    return false;
}
