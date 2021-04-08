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

            return handleHiveMindHostApiRequest(req->getId(), message, *hivemindHostApiReq);
        }
    }

    m_logger.log(LogLevel::Warn, "Received Unknown message in api request handler");
    return false;
}

bool HiveMindHostApiRequestHandler::handleHiveMindHostApiRequest(
    uint16_t requestId, const MessageDTO& message, const HiveMindHostApiRequestDTO& request) {

    if (std::holds_alternative<BytesDTO>(request.getRequest())) {
        return m_hostQueue.push(message);
    }

    if (const auto* neighborReq = std::get_if<GetNeighborRequestDTO>(&request.getRequest())) {
        uint16_t id = neighborReq->getNeighborId();
        std::optional<RelativePosition> pos = m_interloc.getRobotPosition(id);
        NeighborPositionDTO posDTO(pos->m_distance, pos->m_relativeOrientation,
                                   pos->m_isInLineOfSight);

        GetNeighborResponseDTO neighborResp(id, posDTO);
        HiveMindHostApiResponseDTO hmResp(neighborResp);
        ResponseDTO resp(requestId, hmResp);
        MessageDTO msg(m_bsp.getUUId(), message.getSourceId(), resp);
        return m_hostQueue.push(msg);
    }

    if (std::holds_alternative<GetNeighborsListRequestDTO>(request.getRequest())) {

        GetNeighborsListResponseDTO neighborResp(NULL, 0);
        const PositionsTable& posTable = m_interloc.getPositionTable();

        neighborResp.setRawNeighborsLength(posTable.m_positionsLength);
        auto rawNeighbors = neighborResp.getRawNeighbors(posTable.m_positionsLength);
        for (uint16_t i = 0; i < neighborResp.getNeighborsLength(); i++) {
            rawNeighbors[i] = posTable.m_positions[i].m_robotId;
        }

        HiveMindHostApiResponseDTO hmResp(neighborResp);
        ResponseDTO resp(requestId, hmResp);
        MessageDTO msg(m_bsp.getUUId(), message.getSourceId(), resp);
        return m_hostQueue.push(msg);
    }

    m_logger.log(LogLevel::Warn, "Received unknown hivemind host api");
    return false;
}
