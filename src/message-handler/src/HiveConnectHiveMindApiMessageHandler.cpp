#include "HiveConnectHiveMindApiMessageHandler.h"

HiveConnectHiveMindApiMessageHandler::HiveConnectHiveMindApiMessageHandler(
    const IBSP& bsp,
    ICircularQueue<MessageDTO>& hostQueue,
    ICircularQueue<MessageDTO>& remoteQueue,
    ILogger& logger) :
    m_bsp(bsp), m_hostQueue(hostQueue), m_remoteQueue(remoteQueue), m_logger(logger) {}

bool HiveConnectHiveMindApiMessageHandler::handleMessage(const HiveConnectHiveMindApiDTO& message) {

    // TODO: chekc if we want to filter
    if (const auto* req = std::get_if<GetAgentsListRequestDTO>(&message.getMessage())) {
        m_logger.log(
            LogLevel::Warn,
            "Received Request in HiveConnectMessage handler, should only receive response");
        // TODO: What am I supposed to do here?
        MessageDTO msgDTO(1, 1, message);
        m_remoteQueue.push(msgDTO);
    }
    if (const auto* req = std::get_if<GetAgentsListResponseDTO>(&message.getMessage())) {
        HiveMindHostApiResponseDTO apiResponse(*req);
        ResponseDTO response(message.getMessageId(), apiResponse);
        MessageDTO msgDTO(1, 1, message);
        return m_hostQueue.push(msgDTO);
    }
    return false;
}
