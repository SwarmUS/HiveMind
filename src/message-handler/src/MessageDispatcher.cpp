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

bool MessageDispatcher::deserializeAndDispatch() {
    MessageDTO message;
    if (m_deserializer.deserializeFromStream(message)) {

        // Message is for local use
        if (message.getDestinationId() == SettingsContainer::getUUID()) {
            const std::variant<std::monostate, RequestDTO, ResponseDTO>& variantMsg =
                message.getMessage();

            if (const RequestDTO* req = std::get_if<RequestDTO>(&variantMsg)) {

                // Type checking hell
                const std::variant<std::monostate, UserCallRequestDTO>& variantReq =
                    req->getRequest();
                if (const UserCallRequestDTO* uReq = std::get_if<UserCallRequestDTO>(&variantReq)) {

                    // Sending to the appropriate destination
                    UserCallDestinationDTO userCallDestination = uReq->getDestination();
                    switch (userCallDestination) {
                    case UserCallDestinationDTO::BUZZ:
                        m_logger.log(LogLevel::Debug, "Message sent to buzz queue");
                        m_buzzOutputQueue.push(message);
                        return true;
                    case UserCallDestinationDTO::HOST:
                        m_logger.log(LogLevel::Debug, "Message sent to host Q");
                        m_hostOutputQueue.push(message);
                        return true;

                    // Discard the message if unknown
                    case UserCallDestinationDTO::UNKNOWN:
                    default:
                        m_logger.log(LogLevel::Warn, "Unknown user call destination");
                        return false;
                    }
                }
                m_logger.log(LogLevel::Warn, "Unknown request");
            }
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
