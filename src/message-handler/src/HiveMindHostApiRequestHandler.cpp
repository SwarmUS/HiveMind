#include "HiveMindHostApiRequestHandler.h"

HiveMindHostApiRequestHandler::HiveMindHostApiRequestHandler(const IBSP& bsp,
                                                             ICircularQueue<MessageDTO>& hostQueue,
                                                             const IInterloc& interloc,
                                                             ILogger& logger) :
    m_bsp(bsp), m_hostQueue(hostQueue), m_interloc(interloc), m_logger(logger) {}

bool HiveMindHostApiRequestHandler::handleRequest(const MessageDTO& message) {

    if (const auto* req = std::get_if<RequestDTO>(&message.getMessage())) {
        if (const auto* hivemindHostApiReq =
                std::get_if<HiveMindHostApiRequestDTO>(&req->getRequest())) {

            return handleHiveMindHostApiRequest(message, *hivemindHostApiReq);
        }
    }

    m_logger.log(LogLevel::Warn, "Received Unknown message in api request handler");
    return false;
}

bool HiveMindHostApiRequestHandler::handleHiveMindHostApiRequest(
    const MessageDTO& message, const HiveMindHostApiRequestDTO& request) {
    if (std::holds_alternative<BytesDTO>(request.getRequest())) {
        return m_hostQueue.push(message);
    }

    m_logger.log(LogLevel::Warn, "Received unknown hivemind host api");
    return false;
}
