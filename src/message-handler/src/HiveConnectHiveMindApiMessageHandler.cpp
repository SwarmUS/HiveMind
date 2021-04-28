#include "HiveConnectHiveMindApiMessageHandler.h"

HiveConnectHiveMindApiMessageHandler::HiveConnectHiveMindApiMessageHandler(
    const IBSP& bsp, ICircularQueue<MessageDTO>& hostQueue, ILogger& logger) :
    m_bsp(bsp), m_hostQueue(hostQueue), m_logger(logger) {}

bool HiveConnectHiveMindApiMessageHandler::handleMessage(const HiveConnectHiveMindApiDTO& message) {

    // TODO: what happens if comes from a remote?
    if (std::holds_alternative<GetAgentsListRequestDTO>(message.getMessage())) {
        m_logger.log(
            LogLevel::Warn,
            "Received Request in HiveConnectMessage handler, should only receive response");
        return false;
    }
    if (const auto* req = std::get_if<GetAgentsListResponseDTO>(&message.getMessage())) {
        HiveMindHostApiResponseDTO apiResponse(*req);
        ResponseDTO response(message.getMessageId(), apiResponse);
        MessageDTO msgDTO(m_bsp.getUUId(), m_bsp.getUUId(), message);
        return m_hostQueue.push(msgDTO);
    }
    return false;
}
