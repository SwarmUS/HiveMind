#include "HiveConnectHiveMindApiMessageHandler.h"

HiveConnectHiveMindApiMessageHandler::HiveConnectHiveMindApiMessageHandler(
    const IBSP& bsp,
    ICircularQueue<MessageDTO>& hostQueue,
    ICircularQueue<MessageDTO>& remoteQueue,
    ILogger& logger) :
    m_bsp(bsp), m_hostQueue(hostQueue), m_remoteQueue(remoteQueue), m_logger(logger) {}

bool HiveConnectHiveMindApiMessageHandlerhandleMessage(
    const HiveConnectHiveMindApiDTO& apiMessage) {

    if (const auto* req = std::get_if<GetAgentsListRequestDTO>(&apiMessage.getMessage())) {
        MessageDTO msgDTO(1, 1, apiMessage);
    }
    if (const auto* req = std::get_if<GetAgentsListResponseDTO>(&apiMessage.getMessage())) {
    }
    return false;
}
