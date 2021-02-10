#include "MessageDispatcher.h"

MessageDispatcher::MessageDispatcher(ICircularQueue<MessageDTO>& buzzOutputQ,
                                     ICircularQueue<MessageDTO>& tcpOutputQ,
                                     ICircularQueue<MessageDTO>& uartOutputQ,
                                     IHiveMindHostDeserializer& deserializer) :
    m_buzzOutputQueue(buzzOutputQ),
    m_tcpOutputQueue(tcpOutputQ),
    m_uartOutputQueue(uartOutputQ),
    m_deserializer(deserializer) {}

bool MessageDispatcher::deserializeAndDispatch() {
    MessageDTO message;
    if (m_deserializer.deserializeFromStream(message)) {

        const std::variant<std::monostate, RequestDTO, ResponseDTO>& variantMsg =
            message.getMessage();

        if (const RequestDTO* req = std::get_if<RequestDTO>(&variantMsg)) {

            const std::variant<std::monostate, UserCallRequestDTO>& variantReq = req->getRequest();

            if (const UserCallRequestDTO* uReq = std::get_if<UserCallRequestDTO>(&variantReq)) {
                UserCallDestinationDTO userCallDestination = uReq->getDestination();
                switch (userCallDestination) {
                case UserCallDestinationDTO::BUZZ:
                    m_buzzOutputQueue.push(message);
                    break;
                case UserCallDestinationDTO::HOST:
                    m_tcpOutputQueue.push(message);
                    break;

                // Discard the message if unknown
                case UserCallDestinationDTO::UNKNOWN:
                default:
                    break;
                }
            }
        }
    }
