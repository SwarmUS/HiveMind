#include "HiveConnectHiveMindApiMessageHandler.h"

HiveConnectHiveMindApiMessageHandler::HiveConnectHiveMindApiMessageHandler(
    ICircularQueue<MessageDTO>& hostQueue, ILogger& logger) :
    m_hostQueue(hostQueue), m_logger(logger) {}

bool HiveConnectHiveMindApiMessageHandler::handleMessage(uint16_t sourceId,
                                                         uint16_t destId,
                                                         const HiveConnectHiveMindApiDTO& message) {

    if (std::holds_alternative<GetAgentsListRequestDTO>(message.getMessage())) {
        m_logger.log(
            LogLevel::Warn,
            "Received Request in HiveConnectMessage handler, should only receive response");
        return false;
    }
    if (const auto* req = std::get_if<GetAgentsListResponseDTO>(&message.getMessage())) {
        HiveMindHostApiResponseDTO apiResponse(*req);
        ResponseDTO response(message.getMessageId(), apiResponse);
        MessageDTO msgDTO(sourceId, destId, message);
        return m_hostQueue.push(msgDTO);
    }
    return false;
}
