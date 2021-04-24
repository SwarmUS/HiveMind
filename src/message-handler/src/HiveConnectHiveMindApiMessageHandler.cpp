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
        MessageDTO msgDTO(1, 1, message);
        m_remoteQueue.push(msgDTO);
    }
    if (const auto* req = std::get_if<GetAgentsListResponseDTO>(&message.getMessage())) {
        HiveMindHostApiResponseDTO apiResponse(*req);
        ResponseDTO response(1, apiResponse);
        MessageDTO msgDTO(1, 1, message);
        m_hostQueue.push(msgDTO);
    }
    return false;
}
